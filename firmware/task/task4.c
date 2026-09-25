/*
 * 任务四: 直行 + 球稳中心 (手动停止)
 */
#include "task4.h"
#include "../source/0.96oled.h"
#include "../source/balance.h"
#include "../source/timer_ms.h"
#include "../source/motor.h"
#include "../source/line_follow.h"
#include "../source/params.h"

#define AB_TIME_MS       8000    /* 8s超时 */

static int g_p=0;
static uint32_t g_t0=0;
static int g_enc_start=0;  /* 起始编码器值 */

extern volatile int counter_1_A;
extern volatile int counter_2_A;

void task4_init(void){g_p=0; g_enc_start=counter_1_A+counter_2_A;}
int  task4_phase(void){return g_p;}

bool task4_update(int cx, bool ok){
    (void)cx;(void)ok;
    switch(g_p){
    case 0:
        motor_init(1U); motor_set_direction(1U,1U);
        motor_init(2U); motor_set_direction(2U,2U);
        Line_Follow_Init();
        Line_Follow_Set_Speed((float)g_t4_speed);
        Line_Follow_Set_KP(g_kp_x100);
        Line_Follow_Set_KD(g_kd_x100);
        balance_set_mode(3);
        balance_set_target_mm(0);
        g_t0=timer_ms_get();
        g_p=1;
        break;
    case 1: {
        Line_Follow_Update();
        break;
    }
    default: {
        extern volatile float target_speed_1;
        extern volatile float target_speed_2;
        target_speed_1=0; target_speed_2=0;
        break;
    }
    }
    return false;
}

void task4_draw_oled(int cx){
    oled_clear_buf();
    char b[14]; int p=0;
    b[p++]='P';b[p++]='0'+g_p;b[p++]=' ';
    int e=(cx-320)*39/100;
    if(e>=0)b[p++]='+';else{b[p++]='-';e=-e;}
    if(e>=100)b[p++]='0'+e/100; if(e>=10)b[p++]='0'+(e/10)%10;
    b[p++]='0'+e%10;b[p++]='m';b[p++]='m';b[p]=0;
    oled_draw_string(0,0,b);
    /* 编码器: R=原始 L=差值(累计) */
    { int raw=counter_1_A+counter_2_A;
      int del=raw-g_enc_start;
      char eb[14]; int ep=0;
      eb[ep++]='R'; if(raw>=1000)eb[ep++]='0'+raw/1000;
      if(raw>=100)eb[ep++]='0'+(raw/100)%10; if(raw>=10)eb[ep++]='0'+(raw/10)%10;
      eb[ep++]='0'+raw%10; eb[ep++]=' ';
      eb[ep++]='D'; if(del>=1000)eb[ep++]='0'+del/1000;
      if(del>=100)eb[ep++]='0'+(del/100)%10; if(del>=10)eb[ep++]='0'+(del/10)%10;
      eb[ep++]='0'+del%10; eb[ep]=0; oled_draw_string(2,0,eb);}
    if(g_p==2)oled_draw_string(3,0,"T4 OK!");
    oled_refresh();
}
