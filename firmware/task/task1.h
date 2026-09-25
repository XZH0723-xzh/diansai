#ifndef TASK1_H
#define TASK1_H
#include <stdbool.h>
void task1_init(void);
bool task1_update(int ball_cx, bool key_ok);
int  task1_phase(void);
void task1_draw_oled(int ball_cx);
#endif
