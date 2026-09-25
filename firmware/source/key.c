/*
 * key.c - 四键轮询 + 编码器GPIOB中断
 */
#include "key.h"
#include "ti_msp_dl_config.h"

volatile int counter_1_A = 0;
volatile int counter_2_A = 0;
static uint8_t g_key_last = 0;

void key_init(void) { g_key_last = 0; }

key_t key_read(void) {
    uint8_t now = 0;
    if (!(DL_GPIO_readPins(key_PORT, key_ok_PIN)   & key_ok_PIN))   now = 1;
    if (!(DL_GPIO_readPins(key_PORT, key_back_PIN) & key_back_PIN)) now = 2;
    if (!(DL_GPIO_readPins(key_PORT, key_up_PIN)   & key_up_PIN))   now = 3;
    if (!(DL_GPIO_readPins(key_PORT, key_down_PIN) & key_down_PIN)) now = 4;
    key_t ret = KEY_NONE;
    if (now && now != g_key_last) ret = (key_t)now;
    g_key_last = now;
    return ret;
}

void GROUP1_IRQHandler(void) {
    switch (DL_GPIO_getPendingInterrupt(GPIOB)) {
    case DC_MOTOR_AA_IIDX: counter_1_A++; break;
    case DC_MOTOR_BA_IIDX: counter_2_A++; break;
    default: break;
    }
    DL_GPIO_clearInterruptStatus(GPIOB, DL_GPIO_getPendingInterrupt(GPIOB));
}
