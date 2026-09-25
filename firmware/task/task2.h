#ifndef TASK2_H
#define TASK2_H
#include <stdbool.h>
void task2_init(void);
bool task2_update(int ball_cx, bool key_ok);
int  task2_phase(void);
void task2_draw_oled(int ball_cx);
#endif
