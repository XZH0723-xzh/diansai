/*
 * task3.h - 任务三: PID驱动球 0→+5→-5→稳定
 */
#ifndef TASK3_H
#define TASK3_H

#include <stdbool.h>

/* 初始化任务三状态 */
void task3_init(void);

/*
 * 每30ms调用一次, 放在主循环中。
 * 返回 true 表示任务已完成。
 * 内部根据阶段自动切换 balance 模式/目标。
 */
bool task3_update(int ball_cx, bool key_ok);

/* 获取当前阶段和计时 */
int  task3_phase(void);
int  task3_timer_ticks(void);   /* 返回30ms计数, 从OK按下开始 */
void task3_draw_oled(int ball_cx);

#endif
