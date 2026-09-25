/*
 * balance.c - PID滚球控制 (三模式 + 陀螺仪前馈)
 *
 * ===== 可调参数(现场标定) =====
 */
#include "balance.h"
#include "ball_uart.h"
#include "gyro.h"
#include "y42_stepper.h"

/* ---- 机械参数 ---- */
#define CENTER          80.0f   /* 电机水平基准角(°) */
#define MM_PER_PX        0.39f  /* 像素→毫米 */
#define CAM_CENTER        320   /* 摄像头中心像素X */

/* ---- PID (运行时可变) ---- */
static float g_kp[4] = {0.051f, 0.058f, 0.052f, 0.10f};
static float g_kd[4] = {0.253f, 0.43f,  0.24f,  0.50f};
#define KI_STABLE       0.01f
#define KI_TASK3_R      0.002f
#define KI_TASK3_L      0.0079f
#define KI_TASK4        0.002f

/* ---- PID通用限制 ---- */
#define DEAD              12    /* 死区(px) */
#define DEAD_VEL           2    /* 死区速度(px/iter) */
#define INTEG_MAX        30.0f  /* 积分上限(°) */
#define MAX_ANGLE       140.0f  /* 电机角度上限(°) */
#define MIN_MOVE          4.0f  /* 最小移动角(°) */
#define MAX_DELTA        20.0f  /* 单次最大移动角(°) */

/* ---- 陀螺仪前馈 ---- */
#define K_FF             0.02f /* 陀螺仪前馈系数(accel×100→°) */

/* ---- 默认电机转速 ---- */
#define PID_SPEED        60.0f  /* PID移动默认转速(RPM) */

/* ================================================================ */

static bool  g_running;
static int   g_prev_cx;
static float g_integ;
static float g_pos = CENTER;
static int   g_target_px = 0;
static float g_speed = PID_SPEED;
static float g_ki = KI_STABLE;
static int   g_mode = 0;          /* 当前PID模式 */
static int   g_gyro_bias = 0;     /* 陀螺仪零偏 */

void balance_init(void)  { g_running = false; g_target_px = 0; g_gyro_bias = gyro_accel_x(); }
void balance_reset_angle(float angle) { g_pos = angle; }
void balance_start(void)  { g_running = true; g_prev_cx = CAM_CENTER; g_integ = 0; g_pos = CENTER; }
void balance_stop(void)   { g_running = false; Y42_Stop(); }
bool balance_running(void) { return g_running; }
float balance_get_angle(void) { return g_pos; }

void balance_set_target_mm(int mm) {
    g_target_px = (int)((float)mm / MM_PER_PX);
    g_integ = 0.0f;
}
void balance_set_speed(float rpm) { g_speed = rpm; }
void balance_set_mode(int mode) {
    g_mode = (mode >= 0 && mode <= 3) ? mode : 0;
    if (g_mode == 1)      g_ki = KI_TASK3_R;
    else if (g_mode == 2) g_ki = KI_TASK3_L;
    else if (g_mode == 3) g_ki = KI_TASK4;
    else                  g_ki = KI_STABLE;
    g_integ = 0.0f;
}
void balance_set_kp(int mode, int32_t kp_x1000) {
    int m = (mode >= 0 && mode <= 3) ? mode : 0;
    g_kp[m] = (float)kp_x1000 * 0.001f;
}
void balance_set_kd(int mode, int32_t kd_x1000) {
    int m = (mode >= 0 && mode <= 3) ? mode : 0;
    g_kd[m] = (float)kd_x1000 * 0.001f;
}

void balance_update(void) {
    if (!g_running) return;
    if (Y42_IsBusy()) return;

    const ball_data_t *b = ball_uart_get();
    int err = b->cx - CAM_CENTER - g_target_px;
    int vel = b->cx - g_prev_cx;
    g_prev_cx = b->cx;

    int abs_e = (err > 0) ? err : -err;
    if (abs_e < DEAD && (vel > -DEAD_VEL && vel < DEAD_VEL)) return;

    g_integ += g_ki * (float)err;
    if (g_integ > INTEG_MAX)  g_integ = INTEG_MAX;
    if (g_integ < -INTEG_MAX) g_integ = -INTEG_MAX;

    /* 陀螺仪前馈: K_FF=0.8, 车加速时补偿倾角 */
    float ff = K_FF * (float)(gyro_accel_x() - g_gyro_bias);
    float out = CENTER + g_kp[g_mode]*(float)err + g_kd[g_mode]*(float)vel + g_integ + ff;
    if (out > 140.0f) out = 140.0f;  /* CCW上限: 80+60 */
    if (out < 65.0f)  out = 65.0f;   /* CW下限 */

    float delta = out - g_pos;
    if (delta > -MIN_MOVE && delta < MIN_MOVE) return;
    if (delta > MAX_DELTA)  delta = MAX_DELTA;
    if (delta < -MAX_DELTA) delta = -MAX_DELTA;

    Y42_Direction dir = (delta > 0.0f) ? Y42_DIR_CCW : Y42_DIR_CW;
    if (delta < 0.0f) delta = -delta;
    g_pos += (dir == Y42_DIR_CCW ? delta : -delta);
    Y42_MoveAngle(delta, g_speed, dir);
}
