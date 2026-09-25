/*
 * line_follow.c - 八路红外连续位置PID巡线 (纯红外, 无陀螺仪)
 *
 * ===== 可调参数 (现场标定) =====
 */
#include "line_follow.h"
#include "three_ir_sensor.h"
#include "motor.h"

/* ---- 巡线速度 ---- */
#define BASE_SPEED_MM_S          600.0f   /* 直行速度 */
#define LOST_SPEED_MM_S          350.0f   /* 丢线搜索速度 */
#define MAX_TARGET_SPEED_MM_S   1200.0f   /* 单轮上限 */

/* ---- 转向 PID (运行时可变) ---- */
static float g_kp = 0.6f;    /* 默认 0.6, 可通过 params 改 */
static float g_kd = 1.0f;    /* 默认 1.0 */
#define MAX_STEER_MM_S           80.0f    /* 最大差速限制 */

/* ---- 丢线搜索 ---- */
#define LOST_STEER_MM_S          200.0f   /* 搜索转向量 */

/*
 * ======================== A点停车线 ========================
 */
#define A_STOP_ARM_DISTANCE_MM         11036.0f
#define CONTROL_PERIOD_S                   0.030f
#define A_STOP_BLACK_THRESHOLD               6U
#define A_STOP_DEBOUNCE_TICKS                3U
#define ODOMETRY_MAX_VALID_SPEED_MM_S    1200.0f
#define B_DEFAULT_DISTANCE_MM            1500.0f

extern volatile float target_speed_1;
extern volatile float target_speed_2;
extern float speed_1;
extern float speed_2;

/* ---- 运行时状态 ---- */
static float   g_base_speed = BASE_SPEED_MM_S;  /* 可运行时修改 */
static int8_t  g_position;         /* -100~+100, 127=丢线 */
static float   g_last_error;       /* 上次误差 (D项) */
static int8_t  g_last_turn;        /* -1=左转 0=居中 +1=右转 */
static uint8_t g_line_seen;        /* 是否曾见线 */

/* ---- A点 ---- */
typedef enum { A_LINE_IDLE = 0, A_LINE_CONFIRMED, A_LINE_STOPPED } A_Line_State;
static A_Line_State a_line_state = A_LINE_IDLE;
static float   travelled_distance_mm;
static float   a_line_confirm_distance_mm;
static uint8_t a_stop_armed;
static uint8_t a_debounce_count;

/* ---- B点 ---- */
static float   b_distance_mm = B_DEFAULT_DISTANCE_MM;
static uint8_t b_stopped;

/* ================================================================ */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
static float absf(float v) { return (v < 0.0f) ? -v : v; }

static uint8_t popcount8(uint8_t x)
{
    uint8_t c = 0U;
    for (int i = 0; i < 8; i++) { if (x & (1U << i)) c++; }
    return c;
}

static void set_targets(float right, float left)
{
    target_speed_1 = clampf(right, 0.0f, MAX_TARGET_SPEED_MM_S);
    target_speed_2 = clampf(left,  0.0f, MAX_TARGET_SPEED_MM_S);
}

static void latch_a_stop(void)
{
    set_targets(0.0f, 0.0f);
    motor_set_direction(1U, 0U);
    motor_set_direction(2U, 0U);
    a_line_state = A_LINE_STOPPED;
}

static void update_odometry(void)
{
    float spd = (absf(speed_1) + absf(speed_2)) * 0.5f;
    if (spd <= ODOMETRY_MAX_VALID_SPEED_MM_S)
        travelled_distance_mm += spd * CONTROL_PERIOD_S;
    if (travelled_distance_mm >= A_STOP_ARM_DISTANCE_MM)
        a_stop_armed = 1U;
}

static uint8_t update_a_line_detector(uint8_t mask)
{
    update_odometry();

    if (a_line_state == A_LINE_STOPPED) { latch_a_stop(); return 1U; }

    if (!a_stop_armed) {
        a_line_state = A_LINE_IDLE; a_debounce_count = 0U; return 0U;
    }

    if (popcount8(mask) >= A_STOP_BLACK_THRESHOLD) {
        if (a_debounce_count < A_STOP_DEBOUNCE_TICKS) a_debounce_count++;
    } else {
        a_debounce_count = 0U;
    }

    if (a_debounce_count >= A_STOP_DEBOUNCE_TICKS) {
        a_line_confirm_distance_mm = travelled_distance_mm;
        a_line_state = A_LINE_CONFIRMED;
    }
    if (a_line_state == A_LINE_CONFIRMED) { latch_a_stop(); return 1U; }
    return 0U;
}

/* ---- 公开 API ---- */

void Line_Follow_Init(void)
{
    Three_IR_Sensor_Init();
    g_position       = 0;
    g_last_error     = 0.0f;
    g_last_turn      = 0;
    g_line_seen      = 0U;
    a_line_state     = A_LINE_IDLE;
    travelled_distance_mm      = 0.0f;
    a_line_confirm_distance_mm = 0.0f;
    a_stop_armed     = 0U;
    a_debounce_count = 0U;
    b_stopped        = 0U;
    set_targets(0.0f, 0.0f);
}

void Line_Follow_Update(void)
{
    /* 采样 (只读一次, mask和position来自同一帧) */
    uint8_t mask = Three_IR_Sensor_Read_Mask();
    g_position   = Three_IR_Sensor_Calc_Position(mask);

    /* A点停车检测 (优先级最高) */
    if (update_a_line_detector(mask) != 0U) return;

    /* B点里程 */
    if (!b_stopped && travelled_distance_mm >= b_distance_mm) b_stopped = 1U;

    /* 丢线处理 */
    if (g_position == 127) {
        if (!g_line_seen) { set_targets(0.0f, 0.0f); return; }
        if (g_last_turn < 0)
            set_targets(LOST_SPEED_MM_S + LOST_STEER_MM_S,
                        LOST_SPEED_MM_S - LOST_STEER_MM_S);
        else if (g_last_turn > 0)
            set_targets(LOST_SPEED_MM_S - LOST_STEER_MM_S,
                        LOST_SPEED_MM_S + LOST_STEER_MM_S);
        else
            set_targets(LOST_SPEED_MM_S, LOST_SPEED_MM_S);
        return;
    }

    g_line_seen = 1U;

    float error = (float)g_position;
    float steer;

    int8_t abs_pos = (g_position > 0) ? g_position : (int8_t)(-g_position);

    if (abs_pos <= 15) {
        /* 死区: 线近中间, 不转向, 和task4一样直走 */
        steer = 0.0f;
        g_last_error = 0.0f;
        g_last_turn  = 0;
    } else {
        /* 偏远了: PID修正 */
        steer = -(g_kp * error + g_kd * (error - g_last_error));
        steer = clampf(steer, -MAX_STEER_MM_S, MAX_STEER_MM_S);
        g_last_error = error;
        g_last_turn  = (steer > 0.0f) ? 1 : ((steer < 0.0f) ? -1 : 0);
    }

    /* 赛道只有右弯: 禁止左转 */
    if (steer > 0.0f) steer = 0.0f;

    set_targets(g_base_speed + steer, g_base_speed - steer);
}

/* ---- 查询接口 ---- */

uint8_t Line_Follow_Get_Mask(void)           { return Three_IR_Sensor_Read_Mask(); }
float   Line_Follow_Get_Error(void)          { return (g_position == 127) ? 0.0f : (float)g_position; }
float   Line_Follow_Get_Distance_MM(void)    { return travelled_distance_mm; }
uint8_t Line_Follow_Is_A_Stop_Armed(void)    { return a_stop_armed; }
uint8_t Line_Follow_Is_A_Stopped(void)       { return (a_line_state == A_LINE_STOPPED) ? 1U : 0U; }
uint8_t Line_Follow_Is_B_Reached(void)       { return b_stopped; }
void    Line_Follow_Set_B_Distance(float mm) { b_distance_mm = mm; b_stopped = 0U; }
void    Line_Follow_Set_Speed(float mm_s)    { g_base_speed = mm_s; }
void    Line_Follow_Set_KP(int32_t kp_x100)  { g_kp = (float)kp_x100 * 0.01f; }
void    Line_Follow_Set_KD(int32_t kd_x100)  { g_kd = (float)kd_x100 * 0.01f; }
