#ifndef TASK5_H
#define TASK5_H
#include <stdbool.h>
void task5_init(void);
bool task5_update(int ball_cx, bool key_ok);
int  task5_phase(void);
void task5_draw_oled(int ball_cx);
#endif
