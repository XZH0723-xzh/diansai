#ifndef MENU_H
#define MENU_H
#include <stdint.h>

typedef enum { STATE_MAIN, STATE_TASK, STATE_RUNNING, STATE_VIEW, STATE_ADJUST } menu_state_t;
typedef enum { TASK_NONE=0, TASK_1, TASK_2, TASK_3, TASK_4, TASK_5, TASK_6 } task_id_t;
typedef enum { BTN_NONE=0, BTN_UP, BTN_DOWN, BTN_OK, BTN_BACK } btn_t;
typedef struct { int32_t kp,kd; } pid_t;
typedef struct { pid_t line,ball; int16_t cx,mm100; } sys_param_t;

void menu_init(void);
void menu_update(btn_t btn);
void menu_draw(void);
menu_state_t menu_state(void);
task_id_t menu_task(void);
sys_param_t* menu_params(void);
#endif
