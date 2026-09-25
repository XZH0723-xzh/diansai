#ifndef IR_SENSOR_H
#define IR_SENSOR_H
#include <stdint.h>

#define IR_MASK_LEFT   0x04U
#define IR_MASK_MIDDLE 0x02U
#define IR_MASK_RIGHT  0x01U

typedef struct { uint8_t left, middle, right, mask; } ir_state_t;

void ir_sensor_init(void);
ir_state_t ir_sensor_read(void);
uint8_t ir_sensor_read_mask(void);
#endif
