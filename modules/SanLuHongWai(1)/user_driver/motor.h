#ifndef MOTOR_H
#define MOTOR_H

/* 圆周率近似值，用于根据车轮直径计算车轮周长。 */
#define PI 3.14

/*
 * 编码器每转一圈累计的A相脉冲数。
 * 当前测速只统计A相上升沿，因此这里必须与实际编码器和减速比匹配。
 */
#define MOTOR_BIANMAQI 260

/* 车轮直径，单位mm。 */
#define MOTOR_WHEEL_D 67

/*
 * ======================== TB6612驱动接线 ========================
 *
 * MSPM0G3507             TB6612
 * PA02        <------->  STBY
 * PB19        <------->  AIN1
 * PA21        <------->  AIN2
 * PA16        <------->  PWMA（电机1/右轮PWM）
 * PB07        <------->  BIN1
 * PA11        <------->  BIN2
 * PB03        <------->  PWMB（电机2/左轮PWM）
 * GND         <------->  GND
 * 3.3V        <------->  VCC（逻辑电源）
 *
 * TB6612电机电源：
 * VM          <------->  11.5V电机电源正极
 * GND         <------->  电机电源负极
 *
 * TB6612输出：
 * AO1/AO2     <------->  电机1（右轮）
 * BO1/BO2     <------->  电机2（左轮）
 *
 * ======================== 编码器接线 ===========================
 *
 * 电机1/右轮编码器：
 * PB22        <------->  A相
 * PB21        <------->  B相
 *
 * 电机2/左轮编码器：
 * PB08        <------->  A相
 * PB09        <------->  B相
 *
 * 编码器VCC接3.3V，编码器GND接系统GND。
 * MSPM0、TB6612、编码器和电机电源必须共地。
 */

#include "ti_msp_dl_config.h"

/*
 * 初始化指定电机：
 * motor_id=1表示右轮M1，motor_id=2表示左轮M2。
 */
void motor_init(uint8_t motor_id);

/*
 * 设置指定电机的PWM比较值。
 * duty有效范围为0~4000；超过4000会自动限制为4000。
 */
void motor_set_duty(uint8_t motor_id, uint32_t duty);

/*
 * 设置TB6612方向输入：
 * direction=0：两个方向脚同为高电平，停止/制动
 * direction=1：IN1高、IN2低
 * direction=2：IN1低、IN2高
 *
 * 当前车辆前进方向：
 * M1右轮使用direction=1，M2左轮使用direction=2。
 */
void motor_set_direction(uint8_t motor_id, uint8_t direction);

#endif
