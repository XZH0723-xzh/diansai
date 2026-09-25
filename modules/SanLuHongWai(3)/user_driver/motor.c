#include "motor.h"

/*
 * 初始化指定电机及其PWM输出。
 *
 * motor_id=1：
 *   使用TB6612的A通道和PWMA，驱动车体右轮M1。
 *
 * motor_id=2：
 *   使用TB6612的B通道和PWMB，驱动车体左轮M2。
 *
 * 初始化过程：
 *   1. 拉高STBY，使能TB6612驱动芯片。
 *   2. 启动对应电机的PWM定时器。
 *   3. 把两个方向脚都置高，使电机先处于停止/制动状态。
 *   4. 把PWM比较值清零，避免初始化瞬间转动。
 *   5. 启动两个电机共用的50ms速度PI定时器及其中断。
 */
void motor_init(uint8_t motor_id)
{
    /* STBY=1时TB6612才允许输出。 */
    DL_GPIO_setPins(DC_MOTOR_STBY_PORT, DC_MOTOR_STBY_PIN);

    if(motor_id == 1){
        /* 启动右轮M1使用的PWMA。 */
        DL_Timer_startCounter(PWMA_INST);

        /* AIN1=1、AIN2=1，先让M1停止。 */
        DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);

        /* M1初始PWM比较值为0。 */
        DL_Timer_setCaptureCompareValue(PWMA_INST, 0, GPIO_PWMA_C1_IDX);
    }
    else if(motor_id == 2){
        /* 启动左轮M2使用的PWMB。 */
        DL_Timer_startCounter(PWMB_INST);

        /* BIN1=1、BIN2=1，先让M2停止。 */
        DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
        DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);

        /* M2初始PWM比较值为0。 */
        DL_Timer_setCaptureCompareValue(PWMB_INST, 0, GPIO_PWMB_C1_IDX);
    }

    /*
     * MOTOR_PID定时器每50ms产生一次LOAD中断。
     * 中断内依次完成M1和M2的测速及增量式PI计算。
     */
    DL_Timer_startCounter(MOTOR_PID_INST);
    NVIC_EnableIRQ(MOTOR_PID_INST_INT_IRQN);
}

/*
 * 设置指定电机的PWM比较值。
 *
 * duty只表示驱动力大小，不表示旋转方向：
 *   duty=0    -> 无PWM驱动
 *   duty=4000 -> 当前配置允许的最大比较值
 *
 * 正反转由motor_set_direction()单独控制。
 */
void motor_set_duty(uint8_t motor_id, uint32_t duty)
{
    /* 防止上层程序给出超过PWM周期范围的比较值。 */
    if(duty > 4000){
        duty = 4000;
    }

    if(motor_id == 1){
        /* 修改右轮M1的PWMA比较值。 */
        DL_Timer_setCaptureCompareValue(PWMA_INST, duty, GPIO_PWMA_C1_IDX);
    }
    else if(motor_id == 2){
        /* 修改左轮M2的PWMB比较值。 */
        DL_Timer_setCaptureCompareValue(PWMB_INST, duty, GPIO_PWMB_C1_IDX);
    }
}

/*
 * 设置TB6612方向输入。
 *
 * direction=0：
 *   IN1=1、IN2=1，电机停止/制动。
 *
 * direction=1：
 *   IN1=1、IN2=0。
 *
 * direction=2：
 *   IN1=0、IN2=1。
 *
 * 因为左右电机镜像安装，同样的车辆前进方向对应不同电平：
 *   M1右轮前进 -> direction=1
 *   M2左轮前进 -> direction=2
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction)
{
    if(motor_id == 1){
        if(direction == 0){
            /* M1：AIN1=1、AIN2=1，停止/制动。 */
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 1){
            /* M1：AIN1=1、AIN2=0，当前安装下为车辆前进。 */
            DL_GPIO_setPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
        else if(direction == 2){
            /* M1：AIN1=0、AIN2=1，当前安装下为车辆后退。 */
            DL_GPIO_clearPins(DC_MOTOR_AIN1_PORT, DC_MOTOR_AIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_AIN2_PORT, DC_MOTOR_AIN2_PIN);
        }
    }
    else if(motor_id == 2){
        if(direction == 0){
            /* M2：BIN1=1、BIN2=1，停止/制动。 */
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 1){
            /* M2：BIN1=1、BIN2=0，当前安装下为车辆后退。 */
            DL_GPIO_setPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_clearPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
        else if(direction == 2){
            /* M2：BIN1=0、BIN2=1，当前安装下为车辆前进。 */
            DL_GPIO_clearPins(DC_MOTOR_BIN1_PORT, DC_MOTOR_BIN1_PIN);
            DL_GPIO_setPins(DC_MOTOR_BIN2_PORT, DC_MOTOR_BIN2_PIN);
        }
    }
}

/*
 * counter_1_A和counter_2_A定义在key.c中。
 * GPIOB组合中断每检测到一次编码器A相上升沿，就把对应计数器加1。
 */
extern uint32_t counter_1_A;

/* 最近一次计算得到的M1右轮实际速度，单位mm/s。 */
float speed_1 = 0;

extern uint32_t counter_2_A;

/* 最近一次计算得到的M2左轮实际速度，单位mm/s。 */
float speed_2 = 0;

/*
 * 根据最近50ms内的编码器脉冲数计算轮速。
 *
 * 计算过程：
 *   1. counter / MOTOR_BIANMAQI = 50ms内车轮转过的圈数
 *   2. PI * MOTOR_WHEEL_D       = 车轮周长，单位mm
 *   3. 乘20                      = 把50ms路程换算成1秒速度
 *
 * 因为1秒/50ms=20，所以最终结果单位为mm/s。
 * 计算完成后必须清零计数器，为下一个50ms测速窗口重新计数。
 */
void calculate_speed(uint8_t motor_id)
{
    if (motor_id == 1) {
        /* 计算M1右轮速度。 */
        speed_1 = (float)counter_1_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20;

        /* 清零本窗口脉冲数，开始下一个50ms测速周期。 */
        counter_1_A = 0;
    }

    if (motor_id == 2) {
        /* 计算M2左轮速度。 */
        speed_2 = (float)counter_2_A / MOTOR_BIANMAQI * PI * MOTOR_WHEEL_D * 20;

        /* 清零本窗口脉冲数，开始下一个50ms测速周期。 */
        counter_2_A = 0;
    }
}

/*
 * 原双电机工程已经调好的增量式PI参数。
 * 三路循迹只改变左右轮目标速度，不修改这两个速度闭环参数。
 */
float kp = 1.2;  // 比例项：响应本次误差相对上次误差的变化
float ki = 0.6;  // 积分项：根据当前误差持续修正PWM

/* ======================== M1右轮PI状态 ======================== */
uint16_t PWM_1_duty = 0;              // 当前M1 PWM比较值
volatile float target_speed_1 = 0;    // M1目标速度，单位mm/s
float last_error_1 = 0;               // M1上一次速度误差
float current_error_1 = 0;            // M1当前速度误差

/* ======================== M2左轮PI状态 ======================== */
uint16_t PWM_2_duty = 0;              // 当前M2 PWM比较值
volatile float target_speed_2 = 0;    // M2目标速度，单位mm/s
float last_error_2 = 0;               // M2上一次速度误差
float current_error_2 = 0;            // M2当前速度误差

/*
 * 执行指定电机的增量式PI速度闭环。
 *
 * 速度误差：
 *   error = target_speed - actual_speed
 *
 * 增量式PI：
 *   ΔPWM = kp × (当前误差 - 上次误差) + ki × 当前误差
 *   新PWM = 旧PWM + ΔPWM
 *
 * 当实际速度低于目标速度时，误差为正，PI通常增加PWM；
 * 当实际速度高于目标速度时，误差为负，PI通常减小PWM。
 */
void DC_MOTOR_PID(uint8_t motor_id)
{
    float error;       // 本次目标速度与实际速度的差
    float duty_delta;  // 本次需要增加或减少的PWM量
    int32_t next_duty; // 使用有符号数计算，避免负修正发生无符号下溢

    if (motor_id == 1) {
        /*
         * 目标速度为0时立即清除PWM和历史误差。
         * 这样重新启动时不会继承停车前的PI误差。
         */
        if (target_speed_1 <= 0.0f) {
            PWM_1_duty = 0;
            current_error_1 = 0.0f;
            last_error_1 = 0.0f;
            motor_set_duty(motor_id, 0);
            return;
        }

        /* 计算M1当前速度误差。 */
        error = target_speed_1 - speed_1;
        current_error_1 = error;

        /* 根据增量式PI公式计算PWM增量。 */
        duty_delta = kp * (current_error_1 - last_error_1) +
                     ki * current_error_1;

        /* 在上一周期PWM基础上叠加本次修正。 */
        next_duty = (int32_t)PWM_1_duty + (int32_t)duty_delta;

        /* 把M1 PWM限制在硬件允许的0~4000范围内。 */
        if (next_duty < 0) {
            next_duty = 0;
        } else if (next_duty > 4000) {
            next_duty = 4000;
        }

        /* 保存本次结果，供下一周期继续进行增量调节。 */
        PWM_1_duty = (uint16_t)next_duty;
        last_error_1 = current_error_1;

        /* 把计算结果写入右轮PWMA。 */
        motor_set_duty(motor_id, PWM_1_duty);
    }

    if (motor_id == 2) {
        /* M2目标速度为0时，清除PWM和PI历史状态。 */
        if (target_speed_2 <= 0.0f) {
            PWM_2_duty = 0;
            current_error_2 = 0.0f;
            last_error_2 = 0.0f;
            motor_set_duty(motor_id, 0);
            return;
        }

        /* 计算M2当前速度误差。 */
        error = target_speed_2 - speed_2;
        current_error_2 = error;

        /* 根据增量式PI公式计算PWM增量。 */
        duty_delta = kp * (current_error_2 - last_error_2) +
                     ki * current_error_2;

        /* 在上一周期PWM基础上叠加本次修正。 */
        next_duty = (int32_t)PWM_2_duty + (int32_t)duty_delta;

        /* 把M2 PWM限制在硬件允许的0~4000范围内。 */
        if (next_duty < 0) {
            next_duty = 0;
        } else if (next_duty > 4000) {
            next_duty = 4000;
        }

        /* 保存本次结果，供下一周期继续进行增量调节。 */
        PWM_2_duty = (uint16_t)next_duty;
        last_error_2 = current_error_2;

        /* 把计算结果写入左轮PWMB。 */
        motor_set_duty(motor_id, PWM_2_duty);
    }
}

/*
 * MOTOR_PID定时器中断服务函数，每50ms进入一次。
 *
 * 执行顺序：
 *   1. 计算M1右轮实际速度
 *   2. 执行M1速度PI并更新PWMA
 *   3. 计算M2左轮实际速度
 *   4. 执行M2速度PI并更新PWMB
 */
void MOTOR_PID_INST_IRQHandler()
{
    switch (DL_Timer_getPendingInterrupt(MOTOR_PID_INST))
    {
    case DL_TIMER_IIDX_LOAD:
        calculate_speed(1);
        DC_MOTOR_PID(1);
        calculate_speed(2);
        DC_MOTOR_PID(2);
        break;

    // case DL_TIMER_IIDX_COMPARE_0:
    //     status = (status + 3 -1) % 3;
    //     /* code */
    //     break;

    default:
        break;
    }
}
