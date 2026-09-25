/*
 * three_ir_sensor.c - 八路红外循迹 (GPIO6.follow)
 *
 * 逐路读取, 与 OLED 共用 GPIOB 互不干扰。
 */
#include "three_ir_sensor.h"
#include "ti_msp_dl_config.h"

void Three_IR_Sensor_Init(void) {}

uint8_t Three_IR_Sensor_Read_Mask(void)
{
    uint8_t mask = 0U;

    if (DL_GPIO_readPins(follow_PORT, follow_IR_1_PIN) & follow_IR_1_PIN) mask |= 0x01;
    if (DL_GPIO_readPins(follow_PORT, follow_IR_2_PIN) & follow_IR_2_PIN) mask |= 0x02;
    if (DL_GPIO_readPins(follow_PORT, follow_IR_3_PIN) & follow_IR_3_PIN) mask |= 0x04;
    if (DL_GPIO_readPins(follow_PORT, follow_IR_4_PIN) & follow_IR_4_PIN) mask |= 0x08;
    if (DL_GPIO_readPins(follow_PORT, follow_IR_5_PIN) & follow_IR_5_PIN) mask |= 0x10;
    if (DL_GPIO_readPins(follow_PORT, follow_IR_6_PIN) & follow_IR_6_PIN) mask |= 0x20;
    if (DL_GPIO_readPins(follow_PORT, follow_IR_7_PIN) & follow_IR_7_PIN) mask |= 0x40;
    if (DL_GPIO_readPins(follow_PORT, follow_IR_8_PIN) & follow_IR_8_PIN) mask |= 0x80;

    return mask;
}

/*
 * 加权连续位置。算法: Σ(weight_i × bit_i) / Σ(bit_i), 缩放到 ±100。
 * 八路全白 → 丢线, 返回 127。
 */
int8_t Three_IR_Sensor_Get_Position(void)
{
    return Three_IR_Sensor_Calc_Position(Three_IR_Sensor_Read_Mask());
}

int8_t Three_IR_Sensor_Calc_Position(uint8_t mask)
{
    if (mask == 0U) return 127;

    static const int8_t w[8] = {-7, -5, -3, -1, 1, 3, 5, 7};

    int16_t sum = 0;
    int8_t  cnt = 0;

    for (int i = 0; i < 8; i++) {
        if (mask & (uint8_t)(1U << i)) { sum += w[i]; cnt++; }
    }

    return (int8_t)((sum * 100) / (7 * cnt));
}
