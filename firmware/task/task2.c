/*
 * task2.c - 任务二: 巡线一圈
 *
 * 编码器累计里程到达后自动停车。
 * ===== 可调参数 (现场标定) =====
 */
#include "task2.h"
#include "../source/0.96oled.h"
#include "../source/timer_ms.h"
#include "../source/motor.h"
#include "../source/line_follow.h"
#include "../source/params.h"

/* ---- 状态 ---- */
static int      g_phase;       /* 0=起步 1=行驶 2=完成 */
static uint32_t g_t0;          /* 发车时刻(ms) */
static uint32_t g_elapsed;     /* 已用时间(ms) */
static float    g_stop_dist;   /* 停车时里程(mm) */

void task2_init(void)
{
    g_phase     = 0;
    g_elapsed   = 0;
    g_stop_dist = 0.0f;
}

int task2_phase(void) { return g_phase; }

bool task2_update(int ball_cx, bool key_ok)
{
    (void)ball_cx;
    (void)key_ok;

    switch (g_phase) {
    case 0:
        motor_init(1U);
        motor_set_direction(1U, 1U);
        motor_init(2U);
        motor_set_direction(2U, 2U);
        Line_Follow_Init();
        Line_Follow_Set_Speed((float)g_t2_speed);
        Line_Follow_Set_KP(g_kp_x100);
        Line_Follow_Set_KD(g_kd_x100);
        Line_Follow_Set_B_Distance((float)g_t2_lap);
        g_t0    = timer_ms_get();
        g_phase = 1;
        break;

    case 1:
        Line_Follow_Update();
        g_elapsed = timer_ms_get() - g_t0;

        if (Line_Follow_Is_B_Reached()) {
            g_phase      = 2;
            g_stop_dist  = Line_Follow_Get_Distance_MM();
            {
                extern volatile float target_speed_1;
                extern volatile float target_speed_2;
                target_speed_1 = 0.0f;
                target_speed_2 = 0.0f;
            }
            motor_set_direction(1U, 0U);
            motor_set_direction(2U, 0U);
        }
        break;

    default:
        return true;
    }
    return false;
}

void task2_draw_oled(int ball_cx)
{
    (void)ball_cx;
    oled_clear_buf();

    char buf[22];

    /* 标题 + 时间 */
    {
        int p = 0;
        buf[p++] = 'T'; buf[p++] = '2'; buf[p++] = ' ';
        if (g_phase == 2)
            buf[p++] = 'D';
        else if (g_phase == 1)
            buf[p++] = 'R';
        else
            buf[p++] = ' ';
        uint32_t ms = g_elapsed;
        uint16_t s  = ms / 1000U;
        if (s >= 10) buf[p++] = '0' + (s / 10U);
        buf[p++] = '0' + (s % 10U);
        buf[p++] = '.';
        buf[p++] = '0' + ((ms % 1000U) / 100U);
        buf[p++] = 's';
        buf[p] = 0;
        oled_draw_string(0, 0, buf);
    }

    /* 八路红外状态 */
    {
        uint8_t mask = Line_Follow_Get_Mask();
        oled_draw_string(1, 0, "IR:");
        for (int i = 0; i < 8; i++)
            buf[i] = (mask & (1U << i)) ? '1' : '0';
        buf[8] = 0;
        oled_draw_string(1, 24, buf);
    }

    /* 位置 + 距离 */
    {
        float pos  = Line_Follow_Get_Error();
        float dist = (g_phase == 2) ? g_stop_dist
                                    : Line_Follow_Get_Distance_MM();
        uint16_t cm = (uint16_t)(dist / 10.0f);
        int p = 0;
        buf[p++] = 'P'; buf[p++] = ':';
        if (pos > 0) buf[p++] = '+';
        else if (pos < 0) { buf[p++] = '-'; pos = -pos; }
        int ip = (int)pos;
        if (ip >= 100) buf[p++] = '0' + (ip / 100);
        if (ip >= 10)  buf[p++] = '0' + ((ip / 10) % 10);
        buf[p++] = '0' + (ip % 10);
        buf[p++] = ' ';
        buf[p++] = 'D'; buf[p++] = ':';
        if (cm >= 100) buf[p++] = '0' + (cm / 100U);
        if (cm >= 10)  buf[p++] = '0' + ((cm / 10U) % 10U);
        buf[p++] = '0' + (cm % 10U);
        buf[p++] = 'c'; buf[p++] = 'm';
        buf[p] = 0;
        oled_draw_string(2, 0, buf);
    }

    oled_refresh();
}
