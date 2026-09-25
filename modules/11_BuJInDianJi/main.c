#include "ti_msp_dl_config.h"

#include "y42_stepper.h"

/*
 * main.c：Y42闭环步进电机基础测试
 *
 * 上电后的完整动作：
 *   1. 初始化MSPM0G3507时钟、GPIO和TIMG8。
 *   2. 静止等待2秒，给Y42驱动器和独立电源预留稳定时间。
 *   3. 顺时针运行一圈。
 *   4. 停止1秒。
 *   5. 逆时针运行一圈。
 *   6. 停止电机，单片机进入低功耗等待。
 *
 * 当前Y42按16细分计算：
 *   电机整步角为1.8°，一圈需要200个整步；
 *   200整步 × 16细分 = 3200个STEP脉冲/圈。
 *
 * 当前频率为6000脉冲/秒：
 *   转速 = 6000 × 60 ÷ 3200 = 112.5 RPM；
 *   一圈时间 = 3200 ÷ 6000 ≈ 0.533秒。
 */
#define DEMO_PULSE_HZ               (6000U)

/*
 * 毫秒级阻塞延时。
 *
 * CPUCLK_FREQ由SysConfig自动生成，当前通常为32 MHz。
 * delay_cycles()的参数是CPU需要空转的时钟周期数，因此先计算
 * 每毫秒包含多少CPU周期，再乘以需要延时的毫秒数。
 *
 * 该函数只适合当前简单测试；正式工程中若还需要同时执行其他任务，
 * 应改用定时器或系统节拍，避免阻塞CPU。
 */
static void DelayMs(uint32_t ms)
{
    uint32_t cycles_per_ms = CPUCLK_FREQ / 1000U;
    delay_cycles(cycles_per_ms * ms);
}

/*
 * 等待一次定量运动结束。
 *
 * Y42_MoveSteps()是非阻塞函数：调用后TIMG8继续在后台产生STEP脉冲，
 * 对应中断负责累计已经输出的脉冲数。
 *
 * Y42_IsBusy()为true表示目标脉冲还没有全部输出。
 * __WFI()让CPU等待中断，避免在while循环中持续空转。
 */
static void WaitUntilMoveDone(void)
{
    while (Y42_IsBusy()) {
        __WFI();
    }
}

int main(void)
{
    /* 初始化SysConfig生成的时钟、PB11方向GPIO和PA30 STEP定时器。 */
    SYSCFG_DL_init();

    /* 清除Y42软件状态，并使能TIMG8中断。 */
    Y42_Init();

    /* 上电后先等待2秒，期间电机不运动。 */
    DelayMs(2000U);

    /*
     * 输出3200个STEP脉冲，使电机顺时针旋转一圈。
     * 函数返回false表示步数或频率参数无效，此时不会进入等待。
     */
    if (Y42_MoveSteps(
            Y42_PULSES_PER_REV, DEMO_PULSE_HZ, Y42_DIR_CW)) {
        WaitUntilMoveDone();
    }

    /* 正转完成后停止1秒。 */
    DelayMs(1000U);

    /* 用相同的脉冲数和频率反向旋转一圈。 */
    if (Y42_MoveSteps(
            Y42_PULSES_PER_REV, DEMO_PULSE_HZ, Y42_DIR_CCW)) {
        WaitUntilMoveDone();
    }

    /* 再次关闭STEP输出，确保程序结束后电机不再接收脉冲。 */
    Y42_Stop();

    /* 测试任务结束；CPU只等待中断，不再执行其他动作。 */
    while (1) {
        __WFI();
    }
}
