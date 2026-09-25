/*
 * timer_ms.h - 毫秒计时器 (SysTick硬件定时)
 */
#ifndef TIMER_MS_H
#define TIMER_MS_H

#include <stdint.h>

/* 初始化SysTick, 1ms周期 */
void timer_ms_init(void);

/* 返回上电以来的毫秒数 */
uint32_t timer_ms_get(void);

#endif
