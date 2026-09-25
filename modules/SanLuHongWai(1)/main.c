#include "ti_msp_dl_config.h"  // SysConfig生成的外设、GPIO和定时器配置
#include "delay.h"             // 毫秒延时函数
#include "motor.h"             // 双电机驱动、编码器测速和速度PI闭环
#include "line_follow.h"       // 三路红外循迹控制

/*
 * key.c中的按键中断仍然引用status。
 * 三路循迹功能本身不使用此变量，但保留它可避免链接时出现未定义符号。
 */
int status = 0;

/*
 * ======================== 三路红外循迹主程序 ========================
 *
 * 车辆结构：
 *   电机1（M1）= 右轮，车辆前进时使用direction=1
 *   电机2（M2）= 左轮，车辆前进时使用direction=2
 *
 * 三路红外模块接线：
 *   PB23 = L，安装在车体左侧
 *   PB26 = M，安装在车体中间
 *   PB27 = R，安装在车体右侧
 *
 * 模块实测输出极性：
 *   检测到黑线 -> 输出高电平1，模块对应通道灯熄灭
 *   检测到白底 -> 输出低电平0，模块对应通道灯点亮
 *
 * 程序的数据流：
 *   1. 主循环每10ms读取L/M/R。
 *   2. line_follow.c根据三路状态计算左右轮目标速度。
 *   3. motor.c每50ms读取编码器速度。
 *   4. 电机速度PI根据目标速度和实际速度计算PWM。
 * ==================================================================
 */
int main(void)
{
    /*
     * 初始化empty.syscfg中配置的系统时钟、GPIO、PWM和定时器。
     * 必须在使用任何DL_GPIO或DL_Timer接口之前执行。
     */
    SYSCFG_DL_init();

    /*
     * 两个编码器A相都使用GPIOB组合中断：
     *   M1编码器A相 -> PB22
     *   M2编码器A相 -> PB8
     * 中断服务函数位于key.c，负责累加counter_1_A和counter_2_A。
     */
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOB_INT_IRQN);

    /*
     * 初始化右轮M1：
     *   启动PWMA和速度PI定时器；
     *   direction=1对应当前右轮的车辆前进方向。
     */
    motor_init(1U);
    motor_set_direction(1U, 1U);

    /*
     * 初始化左轮M2：
     *   启动PWMB和速度PI定时器；
     *   由于左右电机镜像安装，左轮前进使用direction=2。
     */
    motor_init(2U);
    motor_set_direction(2U, 2U);

    /*
     * 初始化循迹状态。
     * 初始化后两轮目标速度为0，只有读取到有效黑线状态才开始行驶。
     */
    Line_Follow_Init();

    while (1) {
        /*
         * 每10ms读取一次L/M/R并更新左右轮目标速度。
         * 此函数只修改target_speed_1/2，不直接写PWM。
         */
        Line_Follow_Update();
        delay_ms(10U);
    }
}
