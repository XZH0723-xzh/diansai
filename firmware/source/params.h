/*
 * params.h - 脱机可调参数 (运行时修改, 断电丢失)
 *
 * 用法: 任务代码 include 此文件, 直接读 extern 变量。
 *       菜单 PARAM ADJUST 逐位修改。
 */
#ifndef PARAMS_H
#define PARAMS_H

#include <stdint.h>

/* ===== 公共 PID ===== */
extern int32_t g_kp_x100;    /* KP × 100, 默认 60 (即 0.60) */
extern int32_t g_kd_x100;    /* KD × 100, 默认 100 (即 1.00) */

/* ===== T2 巡线一圈 ===== */
extern int32_t g_t2_speed;   /* 直行速度 mm/s, 默认 600 */
extern int32_t g_t2_lap;     /* 一圈里程 mm, 默认 2300 */

/* ===== T3 推球 ===== */
extern int32_t g_t3_kp_r;    /* KP右移 ×1000, 默认 58 (即 0.058) */
extern int32_t g_t3_kd_r;    /* KD右移 ×1000, 默认 430 (即 0.430) */
extern int32_t g_t3_kp_l;    /* KP左移 ×1000, 默认 52 */
extern int32_t g_t3_kd_l;    /* KD左移 ×1000, 默认 240 */

/* ===== T4 直行稳球 ===== */
extern int32_t g_t4_speed;   /* 直行速度 mm/s, 默认 400 */
extern int32_t g_t4_enc;     /* 编码器脉冲数, 默认 6500 */

/* ===== T5 一圈稳球中心 ===== */
extern int32_t g_t5_speed;   /* 直行速度 mm/s, 默认 400 */
extern int32_t g_t5_lap;     /* 一圈里程 mm, 默认 2150 */

/* ===== T6 一圈任意球位 ===== */
extern int32_t g_t6_speed;   /* 直行速度 mm/s, 默认 400 */
extern int32_t g_t6_lap;     /* 一圈里程 mm, 默认 2150 */

/* 初始化(设默认值), 在 main() 开头调用 */
void params_init(void);

/* 菜单用: 获取指定任务的参数列表 */
#define PARAM_TASK_COUNT  5   /* T2~T6共5个任务 */
#define PARAM_MAX_ITEMS   4   /* 每个任务最多4个参数 */

typedef struct {
    const char *name;        /* 显示名, ≤8字符 */
    int32_t    *value;       /* 指向实际变量 */
    int32_t     step;        /* 每次增减量 (1=个位, 10=十位, 100=百位) */
    int32_t     min_val;     /* 最小值 */
    int32_t     max_val;     /* 最大值 */
    uint8_t     decimals;    /* 小数位数 (0=整数, 2=xx.xx, 3=x.xxx) */
    uint8_t     digits;      /* 总显示位数 (含小数点) */
} param_item_t;

/* 返回指定任务的参数列表和数量 */
const param_item_t *params_get_list(int task_id, int *count);

#endif
