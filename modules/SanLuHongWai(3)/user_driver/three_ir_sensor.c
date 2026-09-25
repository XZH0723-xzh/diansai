#include "three_ir_sensor.h"
#include "ti_msp_dl_config.h"

/*
 * 读取一个GPIO输入引脚并把结果统一转换成0或1。
 *
 * DL_GPIO_readPins()返回的是端口寄存器中的位掩码：
 *   目标引脚为高电平时，返回值与pin按位与后不为0；
 *   目标引脚为低电平时，按位与结果为0。
 */
static uint8_t read_input(GPIO_Regs *port, uint32_t pin)
{
    return ((DL_GPIO_readPins(port, pin) & pin) != 0U) ? 1U : 0U;
}

/*
 * 三路GPIO已经在empty.syscfg中配置：
 *   PB23 = IR_TRACK_L，数字输入
 *   PB26 = IR_TRACK_M，数字输入
 *   PB27 = IR_TRACK_R，数字输入
 *
 * 模块输出端带有上拉电阻，所以不需要再开启MSPM0内部上下拉。
 * 此函数保留为统一初始化接口，后续若需要滤波或自检可在这里扩展。
 */
void Three_IR_Sensor_Init(void)
{
}

/*
 * 完成一次三路传感器采样。
 *
 * 单独读取三路后，再把状态组合为：
 *   mask = (L << 2) | (M << 1) | R
 *
 * 这样mask的二进制显示顺序就是L/M/R，例如：
 *   L=1、M=1、R=0 -> mask=0b110
 */
Three_IR_State Three_IR_Sensor_Read(void)
{
    Three_IR_State state;

    /* 读取车体左侧传感器PB23。 */
    state.left = read_input(IR_TRACK_PORT, IR_TRACK_L_PIN);

    /* 读取车体中间传感器PB26。 */
    state.middle = read_input(IR_TRACK_PORT, IR_TRACK_M_PIN);

    /* 读取车体右侧传感器PB27。 */
    state.right = read_input(IR_TRACK_PORT, IR_TRACK_R_PIN);

    /* 将L/M/R压缩到一个三位掩码中，方便循迹状态机判断。 */
    state.mask = (uint8_t)((state.left << 2U) |
                           (state.middle << 1U) |
                           state.right);

    return state;
}

/*
 * 只需要组合状态时使用此函数。
 * 内部仍会读取一次完整L/M/R，然后仅返回mask成员。
 */
uint8_t Three_IR_Sensor_Read_Mask(void)
{
    return Three_IR_Sensor_Read().mask;
}
