/*
 * ir_sensor.c - 八路红外循迹
 * 适配 GPIO6.follow: IR_1=PB2 ... IR_8=PB17
 */
#include "ir_sensor.h"
#include "ti_msp_dl_config.h"

static uint8_t read_pin(GPIO_Regs *port, uint32_t pin) {
    return (DL_GPIO_readPins(port, pin) & pin) ? 1U : 0U;
}

void ir_sensor_init(void) {}

ir_state_t ir_sensor_read(void) {
    ir_state_t s;
    s.left   = read_pin(follow_PORT, follow_IR_1_PIN);
    s.middle = read_pin(follow_PORT, follow_IR_5_PIN);
    s.right  = read_pin(follow_PORT, follow_IR_8_PIN);
    s.mask   = 0;
    if (read_pin(follow_PORT, follow_IR_1_PIN)) s.mask |= 0x01;
    if (read_pin(follow_PORT, follow_IR_2_PIN)) s.mask |= 0x02;
    if (read_pin(follow_PORT, follow_IR_3_PIN)) s.mask |= 0x04;
    if (read_pin(follow_PORT, follow_IR_4_PIN)) s.mask |= 0x08;
    if (read_pin(follow_PORT, follow_IR_5_PIN)) s.mask |= 0x10;
    if (read_pin(follow_PORT, follow_IR_6_PIN)) s.mask |= 0x20;
    if (read_pin(follow_PORT, follow_IR_7_PIN)) s.mask |= 0x40;
    if (read_pin(follow_PORT, follow_IR_8_PIN)) s.mask |= 0x80;
    return s;
}

uint8_t ir_sensor_read_mask(void) {
    return ir_sensor_read().mask;
}
