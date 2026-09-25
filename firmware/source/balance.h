/*
 * balance.h - 滚球平衡控制 (双模式)
 */
#ifndef BALANCE_H
#define BALANCE_H
#include <stdbool.h>
#include <stdint.h>

void balance_init(void);
void balance_start(void);
void balance_stop(void);
bool balance_running(void);
float balance_get_angle(void);
void balance_set_target_mm(int mm);
void balance_set_speed(float rpm);
void balance_set_mode(int mode);  /* 0=稳定, 1=任务三右移, 2=任务三左移, 3=任务四 */
void balance_update(void);
void balance_reset_angle(float angle);
/* 脱机调参: 设置指定mode的KP/KD (kp/kd为×1000的整数, 如58=0.058) */
void balance_set_kp(int mode, int32_t kp_x1000);
void balance_set_kd(int mode, int32_t kd_x1000);

#endif
