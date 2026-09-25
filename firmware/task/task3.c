/*
 * 任务三: PID 推球 0→+5→-5, ≤5s, ±5mm
 *
 * ===== 可调参数 =====
 */
#include "task3.h"
#include "../source/balance.h"
#include "../source/0.96oled.h"
#include "../source/timer_ms.h"
#include "../source/params.h"

#define CAM_CENTER     320     /* 摄像头中心像素X */
#define MM_PER_PX        0.39f  /* 像素→毫米 */

#define TARGET_R        128     /* +5cm err值(px) */
#define TARGET_L       -128     /* -5cm err值(px) */
#define TOL_MM            5     /* 到位容差(mm) */
#define TOL_PX           13     /* 到位容差(px) = 5mm/0.39 */
#define TICK_MAX         166    /* 单阶段超时(30ms*166≈5s) */
#define SETTLE_CNT        10    /* 连续稳住次数 */

static int g_phase = 0, g_settle = 0, g_tick = 0;

void task3_init(void) { g_phase = 0; g_settle = 0; g_tick = 0; }
int  task3_phase(void) { return g_phase; }

bool task3_update(int cx, bool ok) {
    (void)ok;
    int err = cx - CAM_CENTER;
    g_tick++;

    switch (g_phase) {
    case 0: /* 自动启动 → +5cm */
        g_phase = 1; g_settle = 0; g_tick = 0;
        balance_set_kp(1, g_t3_kp_r);
        balance_set_kd(1, g_t3_kd_r);
        balance_set_kp(2, g_t3_kp_l);
        balance_set_kd(2, g_t3_kd_l);
        balance_set_mode(1);
        balance_set_target_mm(50);
        break;
    case 1: /* 球到+5±5mm或超时 → -5cm */
        if (err >= TARGET_R - TOL_PX || g_tick > TICK_MAX) {
            g_phase = 2; g_tick = 0;
            balance_set_mode(2);
            balance_set_target_mm(-50);
        }
        break;
    case 2: /* 球到-5±5mm且稳住或超时 → 完成 */
        if (err <= TARGET_L + TOL_PX || g_tick > TICK_MAX) {
            if (++g_settle > SETTLE_CNT || g_tick > TICK_MAX)
                g_phase = 3;
        } else {
            g_settle = 0;
        }
        break;
    default: return true;
    }
    return false;
}

void task3_draw_oled(int cx) {
    oled_clear_buf();
    char b[14]; int p = 0;
    b[p++] = 'P'; b[p++] = '0' + g_phase; b[p++] = ' ';
    int e = (cx - CAM_CENTER) * (int)(MM_PER_PX * 100) / 100;
    if (e >= 0) b[p++] = '+';
    else { b[p++] = '-'; e = -e; }
    if (e >= 100) b[p++] = '0' + e / 100;
    if (e >= 10)  b[p++] = '0' + (e / 10) % 10;
    b[p++] = '0' + e % 10;
    b[p++] = 'm'; b[p++] = 'm'; b[p] = 0;
    oled_draw_string(0, 0, b);
    if (g_phase == 3) oled_draw_string(3, 0, "T3 OK!");
    oled_refresh();
}
