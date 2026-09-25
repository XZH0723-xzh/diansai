# MSPM0G3507 + Y42 闭环步进电机（STEP/DIR）

本工程只包含 Y42 步进电机控制，不包含直流电机、编码器、灰度、
OLED、舵机、UART、CAN 等其他功能。

## 接线

| MSPM0G3507 天猛星 | Y42 |
| --- | --- |
| PA30 | Stp（橙线） |
| PB11 | Dir（黄线） |
| GND | Gnd（灰线，与电机电源负极共地） |

Y42 的 V+（红线）接独立 10~29 V 电源正极。电源负极接 Y42 Gnd，
同时必须与 MSPM0 GND 共地。

本工程不使用 En、Com、R/A/H、T/B/L。En 可先悬空，Com 不接。

## 引脚冲突检查

- PB11 在原 `10_DC_MOTOR_PID_2` 工程中未使用。
- PA30 在原工程中是 OLED 的 `I2C1_SDA`，存在冲突。
- 本工程已完全移除 OLED/I2C，因此 PA30 可以作为 TIMG8_CCP1 的
  硬件 STEP 输出。
- 将来如果合并工程，OLED 必须改脚，不能让 PA30 同时承担两种功能。

## Y42 参数

代码默认：

- 电机整步角：1.8°
- 细分：16
- 每圈脉冲：200 × 16 = 3200

如果在 Y42 菜单中修改了细分，请同步修改
`user_driver/y42_stepper.h` 中的 `Y42_MICROSTEPS`。

## 上电测试行为

`main.c` 会：

1. 等待 2 秒；
2. 以 800 pulse/s（约 15 RPM）正转 1 圈；
3. 停 1 秒；
4. 以相同速度反转 1 圈；
5. 停止。

第一次测试应卸载机械负载，或让机构处于不会碰撞的位置。若 CW/CCW
方向与实物相反，只需交换 `y42_stepper.c` 里的
`Y42_CW_DIR_LEVEL` 与 `Y42_CCW_DIR_LEVEL`。

## 可调用接口

- `Y42_RunRPM(rpm, direction)`：连续转动，非阻塞。
- `Y42_MoveSteps(steps, pulse_hz, direction)`：定脉冲运动，非阻塞。
- `Y42_MoveAngle(angle_deg, rpm, direction)`：定角度运动，非阻塞。
- `Y42_Stop()`：停止。
- `Y42_IsBusy()`：查询定量运动是否结束。

STEP 使用 TIMG8 硬件 PWM，不是软件延时翻转 GPIO。有限步数由
TIMG8 LOAD 中断累计，因此不会像普通 while 延时发脉冲那样长期阻塞
主循环，后续可与 PID、视觉任务并行。

