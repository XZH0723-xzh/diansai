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
 * 返回最近一次读取的八路状态：
 * bit0~bit7 依次对应 IR_1~IR_8，1表示检测到黑线。
 */
uint8_t Line_Follow_Get_Mask(void);

/*
 * 返回最近一次连续位置：
 *   -100~+100：黑线位置（0=居中，负=偏左，正=偏右）
 *   127：丢线
 */
float Line_Follow_Get_Error(void);

/* 返回左右轮编码器速度积分得到的累计行驶距离，单位mm。 */
float Line_Follow_Get_Distance_MM(void);

/* 返回A点停车检测是否已到达里程使能条件。 */
uint8_t Line_Follow_Is_A_Stop_Armed(void);

/* 返回A点是否已经识别并锁存停车。 */
uint8_t Line_Follow_Is_A_Stopped(void);

/* B点检测 */
uint8_t Line_Follow_Is_B_Reached(void);
void    Line_Follow_Set_B_Distance(float mm);

/* 运行时修改直行速度 (默认600) */
void    Line_Follow_Set_Speed(float mm_s);
/* 运行时修改 PID (KP/KD, ×100 传入整数) */
void    Line_Follow_Set_KP(int32_t kp_x100);
void    Line_Follow_Set_KD(int32_t kd_x100);

#endif
