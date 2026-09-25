/*
 * menu.c - OLED 菜单 (含脱机参数编辑)
 */
#include "menu.h"
#include "0.96oled.h"
#include "gyro.h"
#include "ball_uart.h"
#include "params.h"

/* ===== 参数编辑子状态 ===== */
typedef enum {
    ADJ_PICK_TASK = 0,
    ADJ_PICK_PARAM,
    ADJ_EDIT_DIGIT
} adj_sub_t;

static menu_state_t g_state = STATE_MAIN;
static task_id_t    g_task  = TASK_NONE;
static int          g_cursor = 0;
static sys_param_t  g_param = {{3000,500},{2000,300},320,39};

/* 参数编辑状态 */
static adj_sub_t    g_adj_sub = ADJ_PICK_TASK;
static int          g_adj_task = 2;        /* 当前选的任务 T2~T6 */
static int          g_adj_param_idx = 0;   /* 当前选的参数索引 */
static int          g_adj_digit = 0;       /* 正在编辑的位 (0=最高位) */
static const param_item_t *g_adj_items = NULL;
static int          g_adj_count = 0;

/* 任务名 */
static const char *task_names[] = {"T2 SPD+LAP","T3 PID BAL","T4 SPD+ENC","T5 SPD+LAP","T6 SPD+LAP"};

/* ===== 辅助 ===== */

static void draw_row(int page, const char *text, int sel) {
    if (sel) oled_draw_string_inv(page, 0, text);
    else     oled_draw_string(page, 2, text+1);
}

/* 格式化参数值到buf, 如 "SPD: 0600" 或 "KP: 0.60" */
static void fmt_param_val(char *buf, const param_item_t *it) {
    int p = 0;
    const char *n = it->name;
    while (*n) buf[p++] = *n++;
    buf[p++] = ':'; buf[p++] = ' ';
    int32_t v = *it->value;
    int nd = it->decimals;
    int total = it->digits;
    if (v < 0) { buf[p++] = '-'; v = -v; total--; }
    /* 整数位数 */
    int int_digits = total - nd - (nd > 0 ? 1 : 0);
    /* 输出整数部分 */
    int32_t div = 1;
    for (int i = 1; i < int_digits; i++) div *= 10;
    for (int i = 0; i < int_digits; i++) {
        buf[p++] = '0' + (int)((v / div) % 10);
        div /= 10;
    }
    /* 小数部分 */
    if (nd > 0) {
        buf[p++] = '.';
        int32_t fdiv = 1;
        for (int i = 0; i < nd; i++) fdiv *= 10;
        int32_t frac = v % fdiv;
        fdiv /= 10;
        for (int i = 0; i < nd; i++) {
            buf[p++] = '0' + (int)(frac / fdiv);
            frac %= fdiv;
            fdiv /= 10;
        }
    }
    buf[p] = 0;
}

/* ===== 画各个菜单页 ===== */

static const char *main_items[] = {"* TASK SELECT","* PARAM VIEW","* PARAM ADJUST"};

static void draw_main(void) {
    oled_clear_buf(); oled_draw_string(0,0,"--- MAIN MENU ---");
    for (int i=0;i<3;i++) draw_row(i*2+1,main_items[i],i==g_cursor);
    oled_refresh();
}

static void draw_task(void) {
    oled_clear_buf(); oled_draw_string(0,0,"--- SELECT TASK ---");
    static const char *ti[] = {
        "* T1 RTSP+OSD","* T2 1LAP<=20S","* T3 BALL+-5CM",
        "* T4 AB<=8S","* T5 1LAP BALL","* T6 1LAP BALL+"};
    for (int i=0;i<6;i++) draw_row(i+1,ti[i],i==g_cursor);
    oled_draw_string(7,0,"OK:RUN  BACK:RET"); oled_refresh();
}

static void draw_running(void) {
    oled_clear_buf(); oled_draw_string(0,0,"*** RUNNING ***");
    static const char *ti[] = {"T1","T2","T3","T4","T5","T6"};
    oled_draw_string(2,0,ti[g_task>=1?g_task-1:0]);
    oled_draw_string(5,0,"BACK:EXIT"); oled_refresh();
}

static void draw_view(void) {
    oled_clear_buf(); oled_draw_string(0,0,"--- PARAM VIEW ---");
    char buf[22]; int y10=gyro_yaw(),p=0;
    buf[p++]='Y';buf[p++]=':';
    if(y10<0){buf[p++]='-';y10=-y10;}
    if(y10>=1000)buf[p++]='0'+y10/1000;
    buf[p++]='0'+(y10/100)%10;buf[p++]='0'+(y10/10)%10;buf[p++]='.';
    buf[p++]='0'+y10%10;buf[p]=0; oled_draw_string(1,0,buf);
    const ball_data_t *ball=ball_uart_get(); p=0;
    buf[p++]='B';buf[p++]=':';buf[p++]='X';buf[p++]='=';
    int bx=ball->cx;
    if(bx>=100)buf[p++]='0'+bx/100; if(bx>=10)buf[p++]='0'+(bx/10)%10;
    buf[p++]='0'+bx%10;buf[p++]=' ';buf[p++]='E';buf[p++]='=';
    int be=ball->err_mm; if(be<0){buf[p++]='-';be=-be;}
    if(be>=100)buf[p++]='0'+be/100; if(be>=10)buf[p++]='0'+(be/10)%10;
    buf[p++]='0'+be%10;buf[p++]='m';buf[p++]='m';buf[p]=0;
    oled_draw_string(2,0,buf);
    /* 显示当前 params 里的 PID */
    { char b2[12]; int p2=0;
      b2[p2++]='K';b2[p2++]='P';b2[p2++]=':';
      int kp=g_kp_x100; if(kp>=100)b2[p2++]='0'+kp/100; if(kp>=10)b2[p2++]='0'+(kp/10)%10;
      b2[p2++]='.';b2[p2++]='0'+(kp%10)/5*5;b2[p2]=0; oled_draw_string(4,0,b2);}
    { char b2[12]; int p2=0;
      b2[p2++]='K';b2[p2++]='D';b2[p2++]=':';
      int kd=g_kd_x100; if(kd>=100)b2[p2++]='0'+kd/100; if(kd>=10)b2[p2++]='0'+(kd/10)%10;
      b2[p2++]='.';b2[p2++]='0'+(kd%10)/5*5;b2[p2]=0; oled_draw_string(5,0,b2);}
    oled_refresh();
}

/* ===== 参数编辑画图 ===== */

static void draw_adj_task(void) {
    oled_clear_buf(); oled_draw_string(0,0,"ADJ: PICK TASK");
    for (int i=0;i<5;i++) {
        if (i==g_cursor) oled_draw_string_inv(i+1,0,task_names[i]);
        else oled_draw_string(i+1,0,task_names[i]);
    }
    oled_draw_string(7,0,"OK:SEL BACK:RET"); oled_refresh();
}

static void draw_adj_param(void) {
    oled_clear_buf();
    char b[20];
    int p=0; b[p++]='T'; b[p++]='0'+g_adj_task; b[p++]=' '; b[p++]=0;
    oled_draw_string(0,0,"ADJ: PARAM");
    for (int i=0;i<g_adj_count;i++) {
        fmt_param_val(b,&g_adj_items[i]);
        if (i==g_cursor) oled_draw_string_inv(i+2,0,b);
        else oled_draw_string(i+2,0,b);
    }
    oled_draw_string(7,0,"OK:EDT BACK:RET"); oled_refresh();
}

static void draw_adj_edit(void) {
    oled_clear_buf();
    const param_item_t *it = &g_adj_items[g_adj_param_idx];
    /* 标题 */
    char b[20]; int p=0;
    b[p++]='T';b[p++]='0'+g_adj_task;b[p++]=' ';
    { const char *s = it->name; while (*s) b[p++] = *s++; }
    b[p]=0; oled_draw_string(0,0,b);
    /* 数值, 光标位反白 */
    fmt_param_val(b,it);
    /* 画正常部分 + 光标位反白 */
    int colon_pos = 0;
    for (int i=0;b[i];i++) if(b[i]==':'){colon_pos=i;break;}
    /* 数字部分从 colon_pos+2 开始 */
    int num_start = colon_pos + 2;
    int num_len = 0;
    for (int i=num_start;b[i];i++) num_len++;
    /* 先画正常字符串, 然后在光标位用反白 */
    oled_draw_string(2, 0, b);
    /* 光标位反白叠加 */
    int dig = g_adj_digit;
    if (dig >= num_len) dig = num_len - 1;
    int col = num_start + dig;
    int page = 2;
    char cb[2] = {b[col], 0};
    oled_draw_string_inv(page, col, cb);
    /* 提示 */
    oled_draw_string(5,0,"U/D:+/- OK:NXT");
    oled_draw_string(6,0,"BACK:DONE");
    oled_refresh();
}

/* ===== 参数值修改 ===== */
static void adj_val_change(int delta) {
    const param_item_t *it = &g_adj_items[g_adj_param_idx];
    int32_t v = *it->value;
    int total = it->digits;
    int dig_from_right = total - 1 - g_adj_digit;
    int32_t step = 1;
    for (int i = 0; i < dig_from_right; i++) step *= 10;
    v += delta * step;
    if (v < it->min_val) v = it->min_val;
    if (v > it->max_val) v = it->max_val;
    *it->value = v;
}

/* ===== 主逻辑 ===== */

void menu_init(void) {
    g_state=STATE_MAIN; g_cursor=0; g_task=TASK_NONE;
    g_adj_sub = ADJ_PICK_TASK;
}

void menu_update(btn_t btn) {
    switch (g_state) {
    case STATE_MAIN:
        if(btn==BTN_UP)   g_cursor=(g_cursor>0)?g_cursor-1:2;
        if(btn==BTN_DOWN) g_cursor=(g_cursor<2)?g_cursor+1:0;
        if(btn==BTN_OK) {
            if(g_cursor==0)      { g_state=STATE_TASK; g_cursor=0; }
            else if(g_cursor==1)   g_state=STATE_VIEW;
            else { g_state=STATE_ADJUST; g_cursor=0; g_adj_sub=ADJ_PICK_TASK; }
        }
        break;
    case STATE_TASK:
        if(btn==BTN_UP)   g_cursor=(g_cursor>0)?g_cursor-1:5;
        if(btn==BTN_DOWN) g_cursor=(g_cursor<5)?g_cursor+1:0;
        if(btn==BTN_OK)  { g_task=(task_id_t)(g_cursor+1); g_state=STATE_RUNNING; }
        if(btn==BTN_BACK){ g_state=STATE_MAIN; g_cursor=0; }
        break;
    case STATE_RUNNING:
        if(btn==BTN_BACK){ g_state=STATE_MAIN; g_task=TASK_NONE; }
        break;
    case STATE_VIEW:
        if(btn==BTN_BACK) g_state=STATE_MAIN;
        break;
    case STATE_ADJUST:
        switch (g_adj_sub) {
        case ADJ_PICK_TASK:
            if(btn==BTN_UP)   g_cursor=(g_cursor>0)?g_cursor-1:4;
            if(btn==BTN_DOWN) g_cursor=(g_cursor<4)?g_cursor+1:0;
            if(btn==BTN_OK) {
                g_adj_task = g_cursor + 2; /* T2~T6 */
                g_adj_items = params_get_list(g_adj_task, &g_adj_count);
                g_cursor = 0;
                g_adj_sub = ADJ_PICK_PARAM;
            }
            if(btn==BTN_BACK){ g_state=STATE_MAIN; g_cursor=0; }
            break;
        case ADJ_PICK_PARAM:
            if(btn==BTN_UP)   g_cursor=(g_cursor>0)?g_cursor-1:g_adj_count-1;
            if(btn==BTN_DOWN) g_cursor=(g_cursor<g_adj_count-1)?g_cursor+1:0;
            if(btn==BTN_OK) {
                g_adj_param_idx = g_cursor;
                g_adj_digit = 0;
                g_adj_sub = ADJ_EDIT_DIGIT;
            }
            if(btn==BTN_BACK){ g_adj_sub=ADJ_PICK_TASK; g_cursor=0; }
            break;
        case ADJ_EDIT_DIGIT:
            if(btn==BTN_UP)   adj_val_change(1);
            if(btn==BTN_DOWN) adj_val_change(-1);
            if(btn==BTN_OK) {
                /* 跳到下一位 */
                int total = g_adj_items[g_adj_param_idx].digits;
                g_adj_digit++;
                if (g_adj_digit >= total) g_adj_digit = 0;
            }
            if(btn==BTN_BACK){ g_adj_sub=ADJ_PICK_PARAM; g_cursor=g_adj_param_idx; }
            break;
        }
        break;
    }

    /* 重绘 */
    switch(g_state){
    case STATE_MAIN:    draw_main();    break;
    case STATE_TASK:    draw_task();    break;
    case STATE_RUNNING: draw_running(); break;
    case STATE_VIEW:    draw_view();    break;
    case STATE_ADJUST:
        if(g_adj_sub==ADJ_PICK_TASK)       draw_adj_task();
        else if(g_adj_sub==ADJ_PICK_PARAM) draw_adj_param();
        else                               draw_adj_edit();
        break;
    }
}

void menu_draw(void) {
    switch(g_state){
    case STATE_MAIN:    draw_main();    break;
    case STATE_TASK:    draw_task();    break;
    case STATE_RUNNING: draw_running(); break;
    case STATE_VIEW:    draw_view();    break;
    case STATE_ADJUST:  draw_adj_task(); break;
    }
}

menu_state_t menu_state(void) { return g_state; }
task_id_t    menu_task(void)  { return g_task; }
sys_param_t* menu_params(void) { return &g_param; }
