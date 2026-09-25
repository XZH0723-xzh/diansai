#include "y42_stepper.h"

#include "ti_msp_dl_config.h"

/*
 * y42_stepper.c：Y42的STEP/DIR底层驱动
 *
 * STEP脉冲由TIMG8硬件PWM产生，不依靠软件循环翻转GPIO，因此频率更稳定。
 * DIR由PB11普通GPIO输出。
 *
 * 方向电平映射：
 *   DIR=0 -> 程序定义的顺时针；
 *   DIR=1 -> 程序定义的逆时针。
 *
 * “顺/逆时针”与观察电机的方向有关。如果实物方向和程序名称相反，
 * 只需交换下面两个宏的0和1，不要交换STEP和DIR接线。
 */
#define Y42_CW_DIR_LEVEL             (0U)
#define Y42_CCW_DIR_LEVEL            (1U)

/*
 * 改变DIR后等待5微秒，再输出第一个STEP脉冲。
 * 这样可以满足方向信号相对STEP信号的建立时间，避免第一个脉冲方向错误。
 */
#define Y42_DIR_SETUP_US             (5U)

/* 定量运动的目标脉冲总数。变量会在中断中访问，所以使用volatile。 */
static volatile uint32_t g_target_steps;

/* 当前定量运动已经输出的完整脉冲周期数。 */
static volatile uint32_t g_completed_steps;

/* true表示驱动当前处于运行状态。 */
static volatile bool g_busy;

/* true表示连续运行；false表示输出指定数量脉冲后自动停止。 */
static volatile bool g_continuous;

/* 微秒级短延时，主要用于DIR电平建立时间。 */
static void Y42_DelayUs(uint32_t us)
{
    /* 根据当前CPU主频计算1微秒需要等待的时钟周期数。 */
    uint32_t cycles_per_us = CPUCLK_FREQ / 1000000U;

    /* 防止主频配置异常时计算结果为0。 */
    if (cycles_per_us == 0U) {
        cycles_per_us = 1U;
    }

    delay_cycles(cycles_per_us * us);
}

/* 设置PB11的DIR电平，并等待方向信号稳定。 */
static void Y42_SetDirection(Y42_Direction direction)
{
    /* 根据方向枚举选择需要输出的高低电平。 */
    uint32_t level = (direction == Y42_DIR_CCW) ?
        Y42_CCW_DIR_LEVEL : Y42_CW_DIR_LEVEL;

    if (level != 0U) {
        /* PB11输出高电平。 */
        DL_GPIO_setPins(Y42_CTRL_PORT, Y42_CTRL_DIR_PIN);
    } else {
        /* PB11输出低电平。 */
        DL_GPIO_clearPins(Y42_CTRL_PORT, Y42_CTRL_DIR_PIN);
    }

    /* 确保DIR稳定后再启动STEP定时器。 */
    Y42_DelayUs(Y42_DIR_SETUP_US);
}

/*
 * 配置指定频率、50%占空比的STEP方波。
 *
 * pulse_hz单位为脉冲/秒。
 * Y42_STEP_PWM_INST_CLK_FREQ由SysConfig自动生成，当前为1 MHz。
 * 使用生成的时钟宏而不是写死1000000，使以后修改时钟配置时计算仍然正确。
 *
 * 函数只配置定时器，不会立即启动脉冲输出。
 */
static bool Y42_ConfigurePulseFrequency(uint32_t pulse_hz)
{
    /* 一个STEP周期需要的定时器计数值。 */
    uint32_t period_count;

    /* 50%占空比对应的比较值。 */
    uint32_t compare_count;

    /* 超过软件允许范围就拒绝启动。 */
    if ((pulse_hz < Y42_MIN_PULSE_HZ) ||
        (pulse_hz > Y42_MAX_PULSE_HZ)) {
        return false;
    }

    /*
     * 周期计数 = 定时器时钟 ÷ STEP频率。
     * 加pulse_hz/2后再做整数除法，相当于四舍五入，减小频率误差。
     */
    period_count =
        (Y42_STEP_PWM_INST_CLK_FREQ + (pulse_hz / 2U)) / pulse_hz;

    /*
     * TIMG8是16位定时器，最大计数为65535。
     * 最少保留4个计数，确保高电平和低电平都具有足够宽度。
     */
    if ((period_count < 4U) || (period_count > 65535U)) {
        return false;
    }

    /* 比较值取周期的一半，得到约50%占空比。 */
    compare_count = period_count / 2U;

    /* 修改定时器参数前先停止计数，避免输出畸形脉冲。 */
    DL_TimerG_stopCounter(Y42_STEP_PWM_INST);

    /* 设置一个STEP周期的总计数值。 */
    DL_TimerG_setLoadValue(Y42_STEP_PWM_INST, period_count);

    /* 设置PWM翻转点，产生约50%占空比。 */
    DL_TimerG_setCaptureCompareValue(
        Y42_STEP_PWM_INST,
        compare_count,
        GPIO_Y42_STEP_PWM_C1_IDX);

    /* 从0开始计数，并清除之前残留的中断标志。 */
    DL_TimerG_setTimerCount(Y42_STEP_PWM_INST, 0U);
    DL_TimerG_clearInterruptStatus(Y42_STEP_PWM_INST, 0xFFFFFFFFU);

    return true;
}

void Y42_Init(void)
{
    /* 初始化阶段保持STEP定时器停止。 */
    DL_TimerG_stopCounter(Y42_STEP_PWM_INST);

    /* 在开始定量运动前，不允许LOAD事件触发中断。 */
    DL_TimerG_disableInterrupt(
        Y42_STEP_PWM_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);

    /* 清除可能存在的全部定时器中断状态。 */
    DL_TimerG_clearInterruptStatus(Y42_STEP_PWM_INST, 0xFFFFFFFFU);

    /* DIR默认输出低电平，对应程序定义的顺时针方向。 */
    DL_GPIO_clearPins(Y42_CTRL_PORT, Y42_CTRL_DIR_PIN);

    /* 清空全部运动状态。 */
    g_target_steps = 0U;
    g_completed_steps = 0U;
    g_busy = false;
    g_continuous = false;

    /* 清除并使能TIMG8在NVIC中的中断通道。 */
    NVIC_ClearPendingIRQ(Y42_STEP_PWM_INST_INT_IRQN);
    NVIC_EnableIRQ(Y42_STEP_PWM_INST_INT_IRQN);
}

bool Y42_RunRPM(float rpm, Y42_Direction direction)
{
    /* 浮点形式的目标STEP频率。 */
    float pulse_hz_f;

    /* 四舍五入后的整数STEP频率。 */
    uint32_t pulse_hz;

    /* 转速必须为正数，方向由direction单独指定。 */
    if (rpm <= 0.0f) {
        return false;
    }

    /* 脉冲频率 = RPM × 每圈脉冲数 ÷ 60。 */
    pulse_hz_f = rpm * (float) Y42_PULSES_PER_REV / 60.0f;
    pulse_hz = (uint32_t) (pulse_hz_f + 0.5f);

    /* 先终止上一次运动，再装载新的频率参数。 */
    Y42_Stop();
    if (!Y42_ConfigurePulseFrequency(pulse_hz)) {
        return false;
    }

    /* STEP启动前先确定方向。 */
    Y42_SetDirection(direction);

    /* 连续运行不需要目标步数和已完成步数。 */
    g_target_steps = 0U;
    g_completed_steps = 0U;
    g_continuous = true;
    g_busy = true;

    /*
     * 连续转动不需要统计每个脉冲，因此关闭LOAD中断，降低CPU负担。
     * TIMG8启动后会一直输出STEP，直到调用Y42_Stop()。
     */
    DL_TimerG_disableInterrupt(
        Y42_STEP_PWM_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    DL_TimerG_startCounter(Y42_STEP_PWM_INST);
    return true;
}

bool Y42_MoveSteps(uint32_t steps, uint32_t pulse_hz,
                   Y42_Direction direction)
{
    /* 0步没有实际运动意义，直接返回失败。 */
    if (steps == 0U) {
        return false;
    }

    /* 停止旧运动，并检查、配置新的STEP频率。 */
    Y42_Stop();
    if (!Y42_ConfigurePulseFrequency(pulse_hz)) {
        return false;
    }

    /* STEP启动之前设置DIR。 */
    Y42_SetDirection(direction);

    /* 保存定量运动目标，并将完成计数清零。 */
    g_target_steps = steps;
    g_completed_steps = 0U;
    g_continuous = false;
    g_busy = true;

    /*
     * 每完成一个STEP周期产生一次LOAD中断。
     * 中断函数累计脉冲数量，达到steps后自动停止。
     */
    DL_TimerG_enableInterrupt(
        Y42_STEP_PWM_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    NVIC_ClearPendingIRQ(Y42_STEP_PWM_INST_INT_IRQN);
    DL_TimerG_startCounter(Y42_STEP_PWM_INST);
    return true;
}

bool Y42_MoveAngle(float angle_deg, float rpm,
                   Y42_Direction direction)
{
    /* 角度换算得到的浮点脉冲数。 */
    float steps_f;

    /* RPM换算得到的浮点脉冲频率。 */
    float pulse_hz_f;

    /* 四舍五入后的整数脉冲数和频率。 */
    uint32_t steps;
    uint32_t pulse_hz;

    /* 角度和转速都必须为正，反向通过direction表达。 */
    if ((angle_deg <= 0.0f) || (rpm <= 0.0f)) {
        return false;
    }

    /* 脉冲数 = 角度 × 每圈脉冲数 ÷ 360°。 */
    steps_f = angle_deg * (float) Y42_PULSES_PER_REV / 360.0f;

    /* 脉冲频率 = RPM × 每圈脉冲数 ÷ 60。 */
    pulse_hz_f = rpm * (float) Y42_PULSES_PER_REV / 60.0f;

    /* 加0.5后强制转换为整数，实现正数四舍五入。 */
    steps = (uint32_t) (steps_f + 0.5f);
    pulse_hz = (uint32_t) (pulse_hz_f + 0.5f);

    /* 复用定脉冲接口完成定角度运动。 */
    return Y42_MoveSteps(steps, pulse_hz, direction);
}

void Y42_Stop(void)
{
    /* 禁止新的LOAD中断，并停止STEP定时器。 */
    DL_TimerG_disableInterrupt(
        Y42_STEP_PWM_INST, DL_TIMERG_INTERRUPT_LOAD_EVENT);
    DL_TimerG_stopCounter(Y42_STEP_PWM_INST);

    /*
     * 将比较值设为0，使停止后的PWM输出保持非活动状态。
     * 下一次运动会在Y42_ConfigurePulseFrequency()中重新恢复50%占空比。
     */
    DL_TimerG_setCaptureCompareValue(
        Y42_STEP_PWM_INST, 0U, GPIO_Y42_STEP_PWM_C1_IDX);

    /* 清除停止瞬间可能留下的中断标志。 */
    DL_TimerG_clearInterruptStatus(Y42_STEP_PWM_INST, 0xFFFFFFFFU);

    /* 更新软件运行状态。目标数和完成数保留，便于停止后查询。 */
    g_busy = false;
    g_continuous = false;
}

bool Y42_IsBusy(void)
{
    /* 返回当前运动状态，不进行阻塞等待。 */
    return g_busy;
}

uint32_t Y42_GetTargetSteps(void)
{
    /* 返回本次定量运动的目标脉冲数。 */
    return g_target_steps;
}

uint32_t Y42_GetCompletedSteps(void)
{
    /* 返回本次定量运动已经完成的脉冲数。 */
    return g_completed_steps;
}

/*
 * TIMG8中断服务函数。
 *
 * 一个LOAD中断对应一个完整STEP脉冲周期。
 * 只在“定量运动”模式下累计脉冲；连续运行模式已经关闭该中断。
 * 达到目标脉冲数时，在完整周期边界停止，避免产生被截断的脉冲。
 *
 * 函数名Y42_STEP_PWM_INST_IRQHandler由SysConfig宏展开为TIMG8_IRQHandler。
 */
void Y42_STEP_PWM_INST_IRQHandler(void)
{
    /* 读取并自动清除当前最高优先级的TIMG8中断来源。 */
    switch (DL_TimerG_getPendingInterrupt(Y42_STEP_PWM_INST)) {
        case DL_TIMER_IIDX_LOAD:
            /* 只有正在运行且不是连续模式时才进行脉冲计数。 */
            if (g_busy && !g_continuous) {
                g_completed_steps++;

                /* 达到目标数量后立即停止定时器和后续中断。 */
                if (g_completed_steps >= g_target_steps) {
                    DL_TimerG_stopCounter(Y42_STEP_PWM_INST);
                    DL_TimerG_disableInterrupt(
                        Y42_STEP_PWM_INST,
                        DL_TIMERG_INTERRUPT_LOAD_EVENT);

                    /* 使STEP输出回到停止状态。 */
                    DL_TimerG_setCaptureCompareValue(
                        Y42_STEP_PWM_INST,
                        0U,
                        GPIO_Y42_STEP_PWM_C1_IDX);

                    g_busy = false;
                }
            }
            break;

        default:
            break;
    }
}
