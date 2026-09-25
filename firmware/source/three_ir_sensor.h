/*
 * ======================== 八路红外循迹模块 ========================
 *
 * 接线 (GPIO6.follow, 全部 INPUT + PULL_UP):
 *   IR_1 -> PB2     IR_5 -> PB11
 *   IR_2 -> PB4     IR_6 -> PB15
 *   IR_3 -> PB5     IR_7 -> PB16
 *   IR_4 -> PB6     IR_8 -> PB17
 *
 * 安装方向 (车头朝前):
 *   IR_1 位于车体最左侧，IR_8 位于车体最右侧。
 *
 * 极性 (PULL_UP):
 *   黑线吸收红外 → 光敏管截止 → 上拉拉高 → GPIO读1
 *   白底反射红外 → 光敏管导通 → 拉低     → GPIO读0
 *
 * 掩码 bit0=IR_1(最左) ... bit7=IR_8(最右)
 */

#ifndef THREE_IR_SENSOR_H
#define THREE_IR_SENSOR_H

#include <stdint.h>

/* ---- 每路对应的掩码位 ---- */
#define IR_CH0_MASK  (0x01U)   /* IR_1  PB2  最左 */
#define IR_CH1_MASK  (0x02U)   /* IR_2  PB4 */
#define IR_CH2_MASK  (0x04U)   /* IR_3  PB5 */
#define IR_CH3_MASK  (0x08U)   /* IR_4  PB6 */
#define IR_CH4_MASK  (0x10U)   /* IR_5  PB11 */
#define IR_CH5_MASK  (0x20U)   /* IR_6  PB15 */
#define IR_CH6_MASK  (0x40U)   /* IR_7  PB16 */
#define IR_CH7_MASK  (0x80U)   /* IR_8  PB17 最右 */

/* 初始化 (GPIO 由 syscfg 配置, 此处预留扩展) */
void Three_IR_Sensor_Init(void);

/*
 * 读取八路状态, 返回组合掩码。
 * bit0~bit7 依次对应 IR_1 ~ IR_8, 1=检测到黑线。
 */
uint8_t Three_IR_Sensor_Read_Mask(void);

/*
 * 加权连续位置 (内部读传感器)。
 * 返回值: -100 (最左) ~ +100 (最右),  0 = 居中, 127 = 丢线
 */
int8_t Three_IR_Sensor_Get_Position(void);

/*
 * 从已知掩码计算位置 (不读传感器, 避免二次采样不一致)。
 */
int8_t Three_IR_Sensor_Calc_Position(uint8_t mask);

#endif
