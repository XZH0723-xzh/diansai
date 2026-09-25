#ifndef TASK4_H
#define TASK4_H
#include <stdbool.h>
void task4_init(void);
bool task4_update(int ball_cx, bool key_ok);
int  task4_phase(void);
void task4_draw_oled(int ball_cx);
#endif
