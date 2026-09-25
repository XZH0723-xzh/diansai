#include "line_follow.h"
#include "three_ir_sensor.h"
#include "motor.h"

/*
 * ======================== 循迹速度参数 ========================
 * 所有速度单位均为mm/s，最终由motor.c中的速度PI闭环执行。
 */

/* 中间传感器压线时，左右轮共同采用的直行目标速度。 */
#define BASE_SPEED_MM_S          900.0f

/* 三路全白且曾经见过黑线时，丢线搜索使用的基础速度。 */
#define LOST_SPEED_MM_S          320.0f

/* L+M或M+R同时压线时采用的较小差速修正量。 */
#define SOFT_STEER_MM_S          300.0f

/* 只有L或只有R压线时采用的较大差速修正量。 */
#define HARD_STEER_MM_S          450.0f

/* 三路全白后，按上次转向方向找线时使用的差速量。 */
#define LOST_STEER_MM_S          170.0f

/* 限制单轮最大目标速度，避免循迹计算给出过大的速度指令。 */
#define MAX_TARGET_SPEED_MM_S    1700.0f

/*
 * ======================== A点停车线识别参数 ========================
 * 赛道中心线一圈约为：2*1.5m + 2*pi*0.5m = 6.14m。
 * 小车刚从A点起步时必须忽略启停线，因此只有累计行驶超过5.2m后，
 * 才允许把后续的L->M顺序触发识别为A点停车线。
 */
#define A_STOP_ARM_DISTANCE_MM         11036.0f

/* 主循环每10ms调用一次Line_Follow_Update。 */
#define CONTROL_PERIOD_S                   0.010f

/* R、M各自必须连续有效2次，即稳定20ms，过滤临界抖动。 */
#define A_LINE_DEBOUNCE_SAMPLES            2U

/* R确认后，M必须在300ms内确认，否则本次顺序作废。 */
#define A_LINE_SEQUENCE_TIMEOUT_TICKS     30U

/*
 * 检测行到小车指定停车测试点的前后补偿距离，单位mm。
 * 当前先按检测到A线后立即停车设置为0；实车测量后可改为正数。
 */
#define A_STOP_FORWARD_OFFSET_MM           0.0f

/* 排除编码器干扰产生的明显不合理速度，避免累计里程突然跳变。 */
#define ODOMETRY_MAX_VALID_SPEED_MM_S    1200.0f

/*
 * target_speed_1和target_speed_2定义在motor.c中：
 *   target_speed_1 = M1右轮目标速度
 *   target_speed_2 = M2左轮目标速度
 *
 * volatile表示这些变量会同时被主循环和定时器中断访问，
 * 编译器每次都必须从内存读取，不能长期缓存在寄存器中。
 */
extern volatile float target_speed_1;
extern volatile float target_speed_2;

/* motor.c每50ms更新一次的左右轮实测速度，单位mm/s。 */
extern float speed_1;
extern float speed_2;

/* 保存最近一次L/M/R组合状态，bit2~bit0依次为L/M/R。 */
static uint8_t current_mask = 0U;

/* 保存供调试使用的离散位置误差。 */
static float current_error = 0.0f;

/*
 * 保存最近一次有效转向方向：
 *   -1 = 最近一次向左修正
 *    0 = 最近一次保持居中
 *   +1 = 最近一次向右修正
 */
static int8_t last_turn = 0;

/*
 * 记录上电后是否曾经检测到有效黑线：
 *   0 = 从未见线，三路全白时停车，防止盲目起步
 *   1 = 曾经见线，三路全白时允许按照历史方向搜索
 */
static uint8_t line_seen = 0U;

/* A点检测状态：先等待R，再等待M，最后锁存停车。 */
typedef enum {
    A_LINE_WAIT_RIGHT = 0,
    A_LINE_WAIT_MIDDLE,
    A_LINE_CONFIRMED,
    A_LINE_STOPPED
} A_Line_State;

static A_Line_State a_line_state = A_LINE_WAIT_RIGHT;
static float travelled_distance_mm = 0.0f;
static float a_line_confirm_distance_mm = 0.0f;
static uint8_t a_stop_armed = 0U;
static uint8_t a_right_stable_count = 0U;
static uint8_t a_middle_stable_count = 0U;
static uint16_t a_sequence_ticks = 0U;

/* 把value限制在minimum与maximum之间。 */
static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/* 不依赖数学库的浮点绝对值。 */
static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

/*
 * 统一写入左右轮目标速度：
 *   right_speed -> target_speed_1 -> M1右轮
 *   left_speed  -> target_speed_2 -> M2左轮
 *
 * 两轮速度都会被限制在0~800mm/s。
 * 本函数只修改速度目标，不直接修改PWM占空比。
 */
static void set_targets(float right_speed, float left_speed)
{
    target_speed_1 = clampf(right_speed, 0.0f, MAX_TARGET_SPEED_MM_S);
    target_speed_2 = clampf(left_speed, 0.0f, MAX_TARGET_SPEED_MM_S);
}

/*
 * 锁存停车并把两个TB6612方向输入切换到制动状态。
 * 一旦进入A_LINE_STOPPED，后续主循环不会因为传感器离开横线而重新启动。
 */
static void latch_a_stop(void)
{
    set_targets(0.0f, 0.0f);
    motor_set_direction(1U, 0U);
    motor_set_direction(2U, 0U);
    a_line_state = A_LINE_STOPPED;
}

/*
 * 使用左右轮实测速度的平均值累计车体中心行驶距离。
 * Line_Follow_Update每10ms调用一次，因此本周期距离=速度*0.010s。
 */
static void update_odometry(void)
{
    float right_speed = absf(speed_1);
    float left_speed = absf(speed_2);
    float center_speed = (right_speed + left_speed) * 0.5f;

    if (center_speed <= ODOMETRY_MAX_VALID_SPEED_MM_S) {
        travelled_distance_mm += center_speed * CONTROL_PERIOD_S;
    }

    if (travelled_distance_mm >= A_STOP_ARM_DISTANCE_MM) {
        a_stop_armed = 1U;
    }
}

/*
 * A点停车线顺序检测器。
 *
 * D到A方向固定，实测停车线会先经过R再经过M，而L可能完全碰不到。
 * 因此不再要求瞬间出现111，而是在300ms窗口内锁存R和M的先后事件。
 * 返回1表示已经锁存停车，本周期不应继续执行普通循迹控制。
 */
static uint8_t update_a_line_detector(uint8_t mask)
{
    update_odometry();

    if (a_line_state == A_LINE_STOPPED) {
        latch_a_stop();
        return 1U;
    }

    /* 尚未走完大部分赛道时，忽略A点启停线和普通弯道中的L/M变化。 */
    if (a_stop_armed == 0U) {
        a_line_state = A_LINE_WAIT_RIGHT;
        a_right_stable_count = 0U;
        a_middle_stable_count = 0U;
        a_sequence_ticks = 0U;
        return 0U;
    }

    /* 如果三路恰好同时压线，已经使能后可以直接确认A点。 */
    if (mask == 0x07U) {
        a_line_confirm_distance_mm = travelled_distance_mm;
        a_line_state = A_LINE_CONFIRMED;
    }

    if (a_line_state == A_LINE_WAIT_RIGHT) {
        /* 必须先出现R=1且M=0，防止普通居中循迹的M信号被当成第二步。 */
        if (((mask & THREE_IR_RIGHT_MASK) != 0U) &&
            ((mask & THREE_IR_MIDDLE_MASK) == 0U)) {
            if (a_right_stable_count < A_LINE_DEBOUNCE_SAMPLES) {
                a_right_stable_count++;
            }
        } else {
            a_right_stable_count = 0U;
        }

        if (a_right_stable_count >= A_LINE_DEBOUNCE_SAMPLES) {
            a_line_state = A_LINE_WAIT_MIDDLE;
            a_middle_stable_count = 0U;
            a_sequence_ticks = 0U;
        }
    } else if (a_line_state == A_LINE_WAIT_MIDDLE) {
        a_sequence_ticks++;

        if ((mask & THREE_IR_MIDDLE_MASK) != 0U) {
            if (a_middle_stable_count < A_LINE_DEBOUNCE_SAMPLES) {
                a_middle_stable_count++;
            }
        } else {
            a_middle_stable_count = 0U;
        }

        if (a_middle_stable_count >= A_LINE_DEBOUNCE_SAMPLES) {
            a_line_confirm_distance_mm = travelled_distance_mm;
            a_line_state = A_LINE_CONFIRMED;
        } else if (a_sequence_ticks > A_LINE_SEQUENCE_TIMEOUT_TICKS) {
            /* 超时说明只是普通循迹修正，重新等待下一次R事件。 */
            a_line_state = A_LINE_WAIT_RIGHT;
            a_right_stable_count = 0U;
            a_middle_stable_count = 0U;
            a_sequence_ticks = 0U;
        }
    }

    if (a_line_state == A_LINE_CONFIRMED) {
        if ((travelled_distance_mm - a_line_confirm_distance_mm) >=
            A_STOP_FORWARD_OFFSET_MM) {
            latch_a_stop();
            return 1U;
        }
    }

    return 0U;
}

/* 左右轮目标速度相同，车辆保持直行。 */
static void drive_straight(void)
{
    set_targets(BASE_SPEED_MM_S, BASE_SPEED_MM_S);
}

/*
 * 向左修正：
 *   M1右轮加速，M2左轮减速，左右轮形成差速后车头向左转。
 *
 * 注意：向左修正不是让左轮更快；
 * 对普通两轮差速底盘，外侧的右轮更快才会向左转。
 */
static void steer_left(float correction)
{
    set_targets(BASE_SPEED_MM_S + correction,
                BASE_SPEED_MM_S - correction);
    last_turn = -1;
}

/*
 * 向右修正：
 *   M1右轮减速，M2左轮加速，左右轮形成差速后车头向右转。
 */
static void steer_right(float correction)
{
    set_targets(BASE_SPEED_MM_S - correction,
                BASE_SPEED_MM_S + correction);
    last_turn = 1;
}

/*
 * 清除循迹历史状态。
 * 初始化时目标速度设为0，防止小车上电后在没有识别黑线时直接运动。
 */
void Line_Follow_Init(void)
{
    Three_IR_Sensor_Init();
    current_mask = 0U;
    current_error = 0.0f;
    last_turn = 0;
    line_seen = 0U;
    a_line_state = A_LINE_WAIT_RIGHT;
    travelled_distance_mm = 0.0f;
    a_line_confirm_distance_mm = 0.0f;
    a_stop_armed = 0U;
    a_right_stable_count = 0U;
    a_middle_stable_count = 0U;
    a_sequence_ticks = 0U;
    set_targets(0.0f, 0.0f);
}

/*
 * 根据L/M/R三路状态更新左右轮目标速度。
 *
 * 掩码位顺序为L/M/R，1表示当前探头检测到黑线：
 *
 *   010：黑线位于车辆中间，直行
 *   110：黑线稍偏左，轻微左转
 *   100：黑线明显在左，较强左转
 *   011：黑线稍偏右，轻微右转
 *   001：黑线明显在右，较强右转
 *   000：全白；未见过线则停车，运行中丢线则搜索
 *   111：全黑，按停止线或大面积黑区处理
 *   101：左右黑、中间白，不符合普通连续黑线，按异常停车
 */
void Line_Follow_Update(void)
{
    /* 每次调用只采样一次三路传感器，确保三个判断来自同一控制周期。 */
    current_mask = Three_IR_Sensor_Read_Mask();

    /* A点检测优先级最高；一旦停车锁存，禁止普通循迹重新给出速度。 */
    if (update_a_line_detector(current_mask) != 0U) {
        current_error = 0.0f;
        return;
    }

    switch (current_mask) {
    case 0x02U: /* 010：只有M检测到黑线。 */
        current_error = 0.0f;
        line_seen = 1U;
        last_turn = 0;
        drive_straight();
        break;

    case 0x06U: /* 110：L、M检测到黑线，黑线稍偏左。 */
        current_error = -1.0f;
        line_seen = 1U;
        steer_left(SOFT_STEER_MM_S);
        break;

    case 0x04U: /* 100：只有L检测到黑线，黑线明显在左。 */
        current_error = -2.0f;
        line_seen = 1U;
        steer_left(HARD_STEER_MM_S);
        break;

    case 0x03U: /* 011：M、R检测到黑线，黑线稍偏右。 */
        current_error = 1.0f;
        line_seen = 1U;
        steer_right(SOFT_STEER_MM_S);
        break;

    case 0x01U: /* 001：只有R检测到黑线，黑线明显在右。 */
        current_error = 2.0f;
        line_seen = 1U;
        steer_right(HARD_STEER_MM_S);
        break;

    case 0x00U: /* 000：三路都在白色背景上。 */
        if (line_seen == 0U) {
            /*
             * 上电后从未检测到黑线：
             * 目标速度保持0，避免小车在未知方向盲目行驶。
             */
            current_error = 0.0f;
            set_targets(0.0f, 0.0f);
        } else if (last_turn < 0) {
            /*
             * 丢线前最后一次向左修正：
             * 继续让右轮较快、左轮较慢，低速向左寻找黑线。
             */
            current_error = -3.0f;
            set_targets(LOST_SPEED_MM_S + LOST_STEER_MM_S,
                        LOST_SPEED_MM_S - LOST_STEER_MM_S);
        } else if (last_turn > 0) {
            /*
             * 丢线前最后一次向右修正：
             * 继续让左轮较快、右轮较慢，低速向右寻找黑线。
             */
            current_error = 3.0f;
            set_targets(LOST_SPEED_MM_S - LOST_STEER_MM_S,
                        LOST_SPEED_MM_S + LOST_STEER_MM_S);
        } else {
            /*
             * 丢线前处于居中状态，没有可靠的搜索方向：
             * 两轮以较低的相同速度短暂向前搜索。
             */
            current_error = 0.0f;
            set_targets(LOST_SPEED_MM_S, LOST_SPEED_MM_S);
        }
        break;

    case 0x07U:
        /*
         * 尚未到达里程使能点时，111可能是刚从A点起步经过启停线。
         * 此时按居中直行处理；里程使能后，前面的A点检测器会优先停车。
         */
        current_error = 0.0f;
        line_seen = 1U;
        last_turn = 0;
        drive_straight();
        break;

    case 0x05U: /* 101：左右黑、中间白，视为分叉或异常状态。 */
    default:
        current_error = 0.0f;
        set_targets(0.0f, 0.0f);
        break;
    }
}

/* 返回最近一次三路传感器组合状态。 */
uint8_t Line_Follow_Get_Mask(void)
{
    return current_mask;
}

/* 返回最近一次循迹离散误差，主要用于调试和数据显示。 */
float Line_Follow_Get_Error(void)
{
    return current_error;
}

/* 返回编码器积分得到的累计行驶距离，便于实车标定使能点。 */
float Line_Follow_Get_Distance_MM(void)
{
    return travelled_distance_mm;
}

/* 返回A点停车检测是否已经根据累计里程使能。 */
uint8_t Line_Follow_Is_A_Stop_Armed(void)
{
    return a_stop_armed;
}

/* 返回是否已经识别A点并锁存停车。 */
uint8_t Line_Follow_Is_A_Stopped(void)
{
    return (a_line_state == A_LINE_STOPPED) ? 1U : 0U;
}
