/*
 * gyro.h - JY-901B 陀螺仪 中断驱动
 * UART2: PA24=RX PA23=TX 115200
 */
#ifndef GYRO_H
#define GYRO_H

#include "ti_msp_dl_config.h"

void gyro_init(void);
int  gyro_updated(void);
int  gyro_roll(void);       /* 角度 ×10 (176=17.6°) */
int  gyro_pitch(void);
int  gyro_yaw(void);
int  gyro_accel_x(void);    /* X加速度 ×100 (98=0.98m/s²) */

#endif
