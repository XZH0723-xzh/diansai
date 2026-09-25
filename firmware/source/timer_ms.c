/*
 * timer_ms.c - SysTick 毫秒计时器
 * CPUCLK = 80MHz, SysTick每1ms中断一次
 */
#include "timer_ms.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t g_ms = 0;

void timer_ms_init(void) {
    SysTick->LOAD = 80000 - 1;         /* 80MHz / 1000 */
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk |
                    SysTick_CTRL_TICKINT_Msk |
                    SysTick_CTRL_ENABLE_Msk;
}

uint32_t timer_ms_get(void) {
    return g_ms;
}

void SysTick_Handler(void) {
    g_ms++;
}
