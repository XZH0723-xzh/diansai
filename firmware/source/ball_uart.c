/*
 * ball_uart.c - K230 球位 UART中断接收
 */
#include "ball_uart.h"
#include <string.h>

static ball_data_t g_ball;
static char g_buf[64];
static int  g_idx;

void ball_uart_init(void) {
    memset(&g_ball, 0, sizeof(g_ball));
    g_idx = 0;
    DL_UART_Main_enableInterrupt(k230_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(k230_INST_INT_IRQN);
}

int ball_uart_updated(void)  { return g_ball.fresh; }
void ball_uart_clear(void)   { g_ball.fresh = 0; }

const ball_data_t* ball_uart_get(void) { return &g_ball; }

void ball_uart_set_cx(int cx) {
    g_ball.cx = cx;
    g_ball.cy = 0;
    g_ball.x  = cx - 10;
    g_ball.y  = 0;
    g_ball.w  = 20;
    g_ball.h  = 20;
    g_ball.err_mm = (cx - 320) * 39 / 100;
    g_ball.fresh = 1;
}

/* 协议解析: $len,14,x,y,w,h,label# */
static void parse(char *buf) {
    int x=0,y=0,w=0,h=0;
    char *p = buf;
    while (*p && *p != '$') p++;
    if (*p != '$') return;
    p++;
    while (*p >= '0' && *p <= '9') p++; p++; /* len */
    while (*p >= '0' && *p <= '9') p++; p++; /* 14 */
    while (*p >= '0' && *p <= '9') { x=x*10+(*p-'0'); p++; } p++;
    while (*p >= '0' && *p <= '9') { y=y*10+(*p-'0'); p++; } p++;
    while (*p >= '0' && *p <= '9') { w=w*10+(*p-'0'); p++; } p++;
    while (*p >= '0' && *p <= '9') { h=h*10+(*p-'0'); p++; }
    g_ball.x = x; g_ball.y = y; g_ball.w = w; g_ball.h = h;
    g_ball.cx = x + w/2; g_ball.cy = y + h/2;
    g_ball.err_mm = (g_ball.cx - 320) * 39 / 100;
    g_ball.fresh = 1;
}

/* 中断喂入字节 */
void ball_uart_feed(uint8_t b) {
    g_buf[g_idx++] = b;
    if (b == '#' || g_idx >= 62) {
        g_buf[g_idx] = 0;
        g_idx = 0;
        parse(g_buf);
    }
}

/* UART1 RX中断 (照抄逐飞) */
void UART1_IRQHandler(void) {
    switch (DL_UART_getPendingInterrupt(UART1)) {
        case DL_UART_IIDX_RX:
            ball_uart_feed((uint8_t)DL_UART_Main_receiveData(k230_INST));
            break;
        default: break;
    }
    DL_UART_clearInterruptStatus(UART1, UART1->CPU_INT.RIS);
}
