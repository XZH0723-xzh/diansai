#ifndef THREE_IR_SENSOR_H
#define THREE_IR_SENSOR_H

#include <stdint.h>

/*
 * ======================== 三路红外模块 ========================
 *
 * 接线：
 *   模块L   -> PB23
 *   模块M   -> PB26
 *   模块R   -> PB27
 *   模块VCC -> 3.3V
 *   模块GND -> MSPM0 GND
 *
 * 安装方向：
 *   L必须位于车体左侧，M位于车体中间，R位于车体右侧。
 *
 * 根据模块资料和实测现象，本工程采用以下极性：
 *   黑线 -> 输出高电平1，对应通道指示灯熄灭
 *   白底 -> 输出低电平0，对应通道指示灯点亮
 *
 * 为了让打印和switch判断直接写成L/M/R顺序，状态掩码定义为：
 *   bit2 = L
 *   bit1 = M
 *   bit0 = R
 *
 * 示例：
 *   0b100：只有左探头L检测到黑线
 *   0b010：只有中探头M检测到黑线
 *   0b001：只有右探头R检测到黑线
 */
#define THREE_IR_LEFT_MASK    (0x04U)
#define THREE_IR_MIDDLE_MASK  (0x02U)
#define THREE_IR_RIGHT_MASK   (0x01U)

/*
 * 保存一次完整的三路读取结果。
 * left/middle/right便于单独查看某一路，mask便于switch统一判断。
 */
typedef struct {
    uint8_t left;
    uint8_t middle;
    uint8_t right;
    uint8_t mask;
} Three_IR_State;

/* 传感器接口初始化；GPIO的真正配置由empty.syscfg完成。 */
void Three_IR_Sensor_Init(void);

/* 同时读取L/M/R，并返回单路状态和组合掩码。 */
Three_IR_State Three_IR_Sensor_Read(void);

/* 只返回组合掩码，bit2~bit0依次为L/M/R。 */
uint8_t Three_IR_Sensor_Read_Mask(void);

#endif
