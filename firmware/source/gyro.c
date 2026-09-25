/*
 * gyro.c - JY-901B UART2中断驱动 (照抄逐飞 isr.c)
 */
#include "gyro.h"

static volatile int g_roll, g_pitch, g_yaw, g_ax;
static volatile int g_updated;

static uint8_t g_buf[11], g_idx, g_has_hdr;

void gyro_init(void) {
    DL_UART_Main_enableInterrupt(jy901b_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(jy901b_INST_INT_IRQN);
}

int gyro_updated(void)  { return g_updated; }
int gyro_roll(void)     { g_updated = 0; return g_roll; }
int gyro_pitch(void)    { g_updated = 0; return g_pitch; }
int gyro_yaw(void)      { g_updated = 0; return g_yaw; }
int gyro_accel_x(void)  { g_updated = 0; return g_ax; }

static void gyro_feed(uint8_t b) {
    if (!g_has_hdr) {
        if (b == 0x55) { g_buf[0] = b; g_idx = 1; g_has_hdr = 1; }
        return;
    }
    g_buf[g_idx++] = b;
    if (g_idx >= 11) {
        g_has_hdr = 0; g_idx = 0;
        uint8_t sum = 0;
        for (int i = 0; i < 10; i++) sum += g_buf[i];
        if (sum != g_buf[10]) return;

        if (g_buf[1] == 0x51) {
            /* 加速度: ax/ay/az × 32768 = 1g=9.8m/s² */
            int16_t ax = (int16_t)((g_buf[3]<<8)|g_buf[2]);
            g_ax = (int)(ax * 9800L / 32768);  /* ×100 m/s² */
        } else if (g_buf[1] == 0x53) {
            int16_t r = (int16_t)((g_buf[3]<<8)|g_buf[2]);
            int16_t p = (int16_t)((g_buf[5]<<8)|g_buf[4]);
            int16_t y = (int16_t)((g_buf[7]<<8)|g_buf[6]);
            g_roll  = (int)(r * 1800L / 32768);
            g_pitch = (int)(p * 1800L / 32768);
            g_yaw   = (int)(y * 1800L / 32768);
        }
        g_updated = 1;
    }
}

/* 照抄逐飞 UART2_IRQHandler */
void UART2_IRQHandler(void) {
    switch (DL_UART_getPendingInterrupt(UART2)) {
        case DL_UART_IIDX_RX:
            gyro_feed((uint8_t)DL_UART_Main_receiveData(jy901b_INST));
            break;
        default: break;
    }
    DL_UART_clearInterruptStatus(UART2, UART2->CPU_INT.RIS);
}
