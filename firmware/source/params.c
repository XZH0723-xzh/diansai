/*
 * params.c - 可调参数定义
 */
#include "params.h"
#include <stddef.h>

/* ===== 实际变量 ===== */
int32_t g_kp_x100  = 60;
int32_t g_kd_x100  = 100;

int32_t g_t2_speed = 600;
int32_t g_t2_lap   = 2300;

int32_t g_t3_kp_r  = 58;
int32_t g_t3_kd_r  = 430;
int32_t g_t3_kp_l  = 52;
int32_t g_t3_kd_l  = 240;

int32_t g_t4_speed = 400;
int32_t g_t4_enc   = 6500;

int32_t g_t5_speed = 400;
int32_t g_t5_lap   = 2150;

int32_t g_t6_speed = 400;
int32_t g_t6_lap   = 2150;

/* ===== 每个任务的参数表 ===== */

static const param_item_t t2_items[] = {
    {"SPD", &g_t2_speed, 10, 50, 999, 0, 3},
    {"LAP", &g_t2_lap,   10, 100, 9999, 0, 4},
    {"KP",  &g_kp_x100,  5,   5,  500, 2, 4},
    {"KD",  &g_kd_x100,  5,   5,  500, 2, 4},
};

static const param_item_t t3_items[] = {
    {"KPR", &g_t3_kp_r, 2, 1, 999, 3, 5},
    {"KDR", &g_t3_kd_r, 5, 1, 999, 3, 5},
    {"KPL", &g_t3_kp_l, 2, 1, 999, 3, 5},
    {"KDL", &g_t3_kd_l, 5, 1, 999, 3, 5},
};

static const param_item_t t4_items[] = {
    {"SPD", &g_t4_speed, 10, 50, 999, 0, 3},
    {"ENC", &g_t4_enc,   100, 100, 99999, 0, 5},
    {"KP",  &g_kp_x100,  5,   5,  500, 2, 4},
    {"KD",  &g_kd_x100,  5,   5,  500, 2, 4},
};

static const param_item_t t5_items[] = {
    {"SPD", &g_t5_speed, 10, 50, 999, 0, 3},
    {"LAP", &g_t5_lap,   10, 100, 9999, 0, 4},
    {"KP",  &g_kp_x100,  5,   5,  500, 2, 4},
    {"KD",  &g_kd_x100,  5,   5,  500, 2, 4},
};

static const param_item_t t6_items[] = {
    {"SPD", &g_t6_speed, 10, 50, 999, 0, 3},
    {"LAP", &g_t6_lap,   10, 100, 9999, 0, 4},
    {"KP",  &g_kp_x100,  5,   5,  500, 2, 4},
    {"KD",  &g_kd_x100,  5,   5,  500, 2, 4},
};

static const struct {
    const param_item_t *items;
    int count;
} g_task_params[PARAM_TASK_COUNT] = {
    {t2_items, 4},
    {t3_items, 4},
    {t4_items, 4},
    {t5_items, 4},
    {t6_items, 4},
};

void params_init(void)
{
    /* 值已在声明时初始化, 此处预留 */
}

const param_item_t *params_get_list(int task_id, int *count)
{
    int idx = task_id - 2;  /* T2→0, T3→1, ... */
    if (idx < 0 || idx >= PARAM_TASK_COUNT) { *count = 0; return NULL; }
    *count = g_task_params[idx].count;
    return g_task_params[idx].items;
}
