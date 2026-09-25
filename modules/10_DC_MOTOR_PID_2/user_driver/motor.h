#ifndef MOTOR_H
#define MOTOR_H

#define PI 3.14

// 编码器线数
#define MOTOR_BIANMAQI 260
// 轮胎直径 mm
#define MOTOR_WHEEL_D 67

// G3507      TB6612
// PA2 <--> STBY
// PB19 <--> AIN1
// PA21 <--> AIN2
// PA16 <--> PWMA
// GND <--> GND
// 3V3 <--> VCC

// TB6612    电源模块
// VM          11.5V
// GND         GND

// TB6612    直流电机1
// AO1<--> M+
// AO2<--> M-

// G3507    直流电机1
// P822 <--> A
// PB21 <--> B
// 3V3 <--> VCC
// GND <--> GND

// 

// 所有的GND都需要连接在一起

#include "ti_msp_dl_config.h"

void motor_init(uint8_t motor_id);
void motor_set_duty(uint8_t motor_id, uint32_t duty);
void motor_set_direction(uint8_t motor_id, uint8_t direction);

#endif // MOTOR_H
