/*
 *  ============ 0.96oled.h ============
 *  SSD1306 0.96寸 OLED 128x64 驱动 (软件I2C)
 *
 *  硬件连接:
 *    SCL -> PA.15
 *    SDA -> PA.16
 *
 *  依赖: TI MSPM0 DriverLib (dl_gpio.h, m0p/dl_core.h)
 */

#ifndef OLED_096_H
#define OLED_096_H

#include "ti_msp_dl_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================================================================
 *                        宏定义
 * =================================================================== */

/* --- 引脚定义 (可根据实际接线修改) --- */
#ifndef OLED_PORT
#define OLED_PORT               GPIOB
#endif

#ifndef OLED_SCL_PIN
#define OLED_SCL_PIN            DL_GPIO_PIN_9
#endif

#ifndef OLED_SDA_PIN
#define OLED_SDA_PIN            DL_GPIO_PIN_8
#endif

/* SSD1306 I2C地址 (7位地址0x3C, 左移1位=0x78) */
#define OLED_I2C_ADDR           0x3C

/* 屏幕尺寸 */
#define OLED_WIDTH              128
#define OLED_HEIGHT             64
#define OLED_PAGES              (OLED_HEIGHT / 8)

/* ===================================================================
 *                        API 函数
 * =================================================================== */

/* 初始化OLED, 必须在SYSCFG_DL_init()之后调用 */
void oled_init(void);

/* 清空显示缓冲区 (不会立即刷新到屏幕, 需调用oled_refresh) */
void oled_clear_buf(void);

/* 将缓冲区内容刷新到OLED屏幕 */
void oled_refresh(void);

/* 在缓冲区指定位置绘制一个6x8字符, 返回下一字符的列坐标 */
uint8_t oled_draw_char(uint8_t page, uint8_t col, char c);

/* 在缓冲区指定位置绘制字符串 */
void oled_draw_string(uint8_t page, uint8_t col, const char *str);
void oled_draw_string_inv(uint8_t page, uint8_t col, const char *str);

#ifdef __cplusplus
}
#endif

#endif /* OLED_096_H */
