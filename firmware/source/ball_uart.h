/*
 * ball_uart.h - K230 UART 球位接收
 */
#ifndef BALL_UART_H
#define BALL_UART_H

#include "ti_msp_dl_config.h"
#include <stdint.h>

typedef struct {
    int16_t x, y, w, h, cx, cy;
    int16_t err_mm;
    uint8_t fresh;
} ball_data_t;

void ball_uart_init(void);
int  ball_uart_updated(void);
void ball_uart_clear(void);
const ball_data_t* ball_uart_get(void);
void ball_uart_set_cx(int cx);    /* 串口收到球数据时注入 */

#endif
