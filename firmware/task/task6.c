/*
 * task6.c - 任务六: 巡线一圈 + 球稳任意指定位置
 *
 * 流程: 选定任务 → 放好球 → 按OK记录位置 → 发车巡线+稳球 → 里程到停车
 */
#include "task6.h"
#include "../source/0.96oled.h"
#include "../source/timer_ms.h"
#include "../source/motor.h"
#include "../source/line_follow.h"
#include "../source/balance.h"
#include "../source/params.h"

/* ---- 状态 ---- */
static int      g_phase;       /* 0=等OK 1=行驶 2=完成 */
static int      g_target_cx;   /* 记录的球目标像素X */
static int      g_target_mm;   /* 记录的球目标偏移(mm) */
static uint32_t g_t0;
static uint32_t g_elapsed;
static float    g_stop_dist;

void task6_init(void)
{
    g_phase     = 0;
    g_target_cx = 320;
    g_target_mm = 0;
    g_elapsed   = 0;
    g_stop_dist = 0.0f;
}

int task6_phase(void) { return g_phase; }

bool task6_update(int ball_cx, bool key_ok)
{
    switch (g_phase) {
    case 0:
        /* 等待用户放好球, 按OK记录位置 */
        if (key_ok) {
            g_target_cx = ball_cx;
            g_target_mm = (int)((float)(ball_cx - 320) * 0.39f);
            /* 初始化电机 + 巡线 + 平衡 */
            motor_init(1U);
            motor_set_direction(1U, 1U);
            motor_init(2U);
            motor_set_direction(2U, 2U);
            Line_Follow_Init();
            Line_Follow_Set_Speed((float)g_t6_speed);
            Line_Follow_Set_KP(g_kp_x100);
            Line_Follow_Set_KD(g_kd_x100);
            Line_Follow_Set_B_Distance((float)g_t6_lap);
            balance_set_mode(3);
            balance_set_target_mm(g_target_mm);
            g_t0    = timer_ms_get();
            g_phase = 1;
        }
        break;

    case 1:
        Line_Follow_Update();
        g_elapsed = timer_ms_get() - g_t0;

        if (Line_Follow_Is_B_Reached()) {
            g_phase     = 2;
            g_stop_dist = Line_Follow_Get_Distance_MM();
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

void task6_draw_oled(int ball_cx)
{
    oled_clear_buf();

    char buf[22];

    /* 标题 + 时间 */
    {
        int p = 0;
        buf[p++] = 'T'; buf[p++] = '6'; buf[p++] = ' ';
        if (g_phase == 2)
            buf[p++] = 'D';
        else if (g_phase == 1)
            buf[p++] = 'R';
        else
            buf[p++] = 'W';
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

    /* 目标位置 + 当前位置 */
    {
        int p = 0;
        buf[p++] = 'T'; buf[p++] = ':';
        int t_mm = g_target_mm;
        if (t_mm >= 0) buf[p++] = '+';
        else { buf[p++] = '-'; t_mm = -t_mm; }
        if (t_mm >= 100) buf[p++] = '0' + (t_mm / 100);
        if (t_mm >= 10)  buf[p++] = '0' + ((t_mm / 10) % 10);
        buf[p++] = '0' + (t_mm % 10);
        buf[p++] = 'm'; buf[p++] = 'm';
        buf[p] = 0;
        oled_draw_string(1, 0, buf);
    }

    /* 实时球偏差 */
    {
        int err = ball_cx - g_target_cx;
        int e_mm = (int)((float)err * 0.39f);
        int p = 0;
        buf[p++] = 'E'; buf[p++] = ':';
        if (e_mm >= 0) buf[p++] = '+';
        else { buf[p++] = '-'; e_mm = -e_mm; }
        if (e_mm >= 100) buf[p++] = '0' + (e_mm / 100);
        if (e_mm >= 10)  buf[p++] = '0' + ((e_mm / 10) % 10);
        buf[p++] = '0' + (e_mm % 10);
        buf[p++] = 'm'; buf[p++] = 'm';
        buf[p] = 0;
        oled_draw_string(2, 0, buf);
    }

    /* 距离 */
    {
        float dist = (g_phase == 2) ? g_stop_dist
                                    : Line_Follow_Get_Distance_MM();
        uint16_t cm = (uint16_t)(dist / 10.0f);
        int p = 0;
        buf[p++] = 'D'; buf[p++] = ':';
        if (cm >= 100) buf[p++] = '0' + (cm / 100U);
        if (cm >= 10)  buf[p++] = '0' + ((cm / 10U) % 10U);
        buf[p++] = '0' + (cm % 10U);
        buf[p++] = 'c'; buf[p++] = 'm';
        buf[p] = 0;
        oled_draw_string(3, 0, buf);
    }

    oled_refresh();
}
