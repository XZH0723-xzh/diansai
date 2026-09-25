/*
 * 菜单 + 任务
 */
#include "ti_msp_dl_config.h"
#include "source/0.96oled.h"
#include "source/ball_uart.h"
#include "source/y42_stepper.h"
#include "source/balance.h"
#include "source/key.h"
#include "source/delay.h"
#include "source/timer_ms.h"
#include "source/menu.h"
#include "source/gyro.h"
#include "source/params.h"
#include "task/task2.h"
#include "task/task3.h"
#include "task/task4.h"
#include "task/task5.h"
#include "task/task6.h"

#define CENTER 80.0f
#define SPEED  150.0f
int status = 0;

static btn_t key_to_btn(key_t k) {
    switch (k) {
    case KEY_UP:   return BTN_UP;
    case KEY_DOWN: return BTN_DOWN;
    case KEY_OK:   return BTN_OK;
    case KEY_BACK: return BTN_BACK;
    default:       return BTN_NONE;
    }
}

int main(void) {
    SYSCFG_DL_init();
    NVIC_EnableIRQ(DC_MOTOR_INT_IRQN);
    DL_TimerG_setCCPOutputDisabled(PWM_8_INST,
        DL_TIMER_CCP_DIS_OUT_LOW, DL_TIMER_CCP_DIS_OUT_LOW);
    DL_TimerG_stopCounter(PWM_8_INST);

    oled_init();
    ball_uart_init();
    key_init();
    Y42_Init();
    balance_init();
    gyro_init();
    timer_ms_init();
    params_init();
    menu_init();
    task2_init(); task3_init(); task4_init(); task5_init(); task6_init();
    delay_ms(2000);

    Y42_MoveAngle(CENTER, 100.0f, Y42_DIR_CCW);
    { uint32_t t0 = timer_ms_get();
      while (Y42_IsBusy()) { if (timer_ms_get()-t0>3000) { Y42_Stop(); break; } }
    }
    delay_ms(2000);
    balance_start();
    balance_set_mode(0);
    balance_set_speed(SPEED);

    while (1) {
        const ball_data_t *b = ball_uart_get();
        int cx = b->cx;
        key_t k = key_read();
        btn_t btn = key_to_btn(k);
        task_id_t task = menu_task();

        if (task == TASK_NONE) {
            balance_update();
            menu_update(btn);
            delay_ms(10);
            continue;
        }

        { static task_id_t last = TASK_NONE;
          if (task != last) {
              if (task == TASK_2) task2_init();
              if (task == TASK_3) task3_init();
              if (task == TASK_4) task4_init();
              if (task == TASK_5) task5_init();
              if (task == TASK_6) task6_init();
              last = task;
          }
          if (k == KEY_BACK) last = TASK_NONE; }

        switch (task) {
        case TASK_2: task2_update(cx, k==KEY_OK); task2_draw_oled(cx); break;
        case TASK_3: task3_update(cx, k==KEY_OK); task3_draw_oled(cx); break;
        case TASK_4: task4_update(cx, k==KEY_OK); task4_draw_oled(cx); break;
        case TASK_5: task5_update(cx, k==KEY_OK); task5_draw_oled(cx); break;
        case TASK_6: task6_update(cx, k==KEY_OK); task6_draw_oled(cx); break;
        default: break;
        }

        balance_update();

        if (k == KEY_BACK) {
            menu_init();
            Y42_Stop();
            balance_stop();
            balance_start();
            balance_set_mode(0);
            balance_set_target_mm(0);
            balance_set_speed(SPEED);
        }

        delay_ms(10);
    }
}
