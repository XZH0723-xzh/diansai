#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <stdint.h>

/*
 * 初始化循迹控制器的历史状态和目标速度。
 * 必须在SYSCFG_DL_init()和电机初始化之后调用一次。
 */
void Line_Follow_Init(void);

/*
 * 读取L/M/R并更新左右轮目标速度。
 * 建议在主循环中每10ms调用一次。
 *
 * 本函数不直接控制PWM：
 *   line_follow.c只计算target_speed_1/target_speed_2；
 *   motor.c中的50ms速度PI闭环再根据编码器反馈计算PWM。
 */
void Line_Follow_Update(void);

/*
 * 返回最近一次读取的三路状态：
 * bit2=L、bit1=M、bit0=R，1表示检测到黑线。
 */
uint8_t Line_Follow_Get_Mask(void);

/*
 * 返回最近一次离散位置误差：
 *   -2：黑线明显在左
 *   -1：黑线稍微偏左
 *    0：黑线居中
 *   +1：黑线稍微偏右
 *   +2：黑线明显在右
 *   -3/+3：已经丢线，正在向左/向右搜索
 */
float Line_Follow_Get_Error(void);

/* 返回左右轮编码器速度积分得到的累计行驶距离，单位mm。 */
float Line_Follow_Get_Distance_MM(void);

/* 返回A点停车检测是否已到达里程使能条件。 */
uint8_t Line_Follow_Is_A_Stop_Armed(void);

/* 返回A点是否已经识别并锁存停车。 */
uint8_t Line_Follow_Is_A_Stopped(void);

#endif
