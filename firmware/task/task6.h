/*
 * task6.h - 任务六: 巡线一圈 + 球稳任意指定位置
 */
#ifndef TASK6_H
#define TASK6_H

#include <stdbool.h>

void task6_init(void);
bool task6_update(int ball_cx, bool key_ok);
int  task6_phase(void);
void task6_draw_oled(int ball_cx);

#endif
