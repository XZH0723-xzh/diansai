# 2026 电赛 H 题 · 车载平衡滚球运动控制系统

全国大学生电子设计竞赛 H 题「车载平衡滚球运动控制系统」完整代码仓库。

小车沿赛道黑线行驶（红外巡线 + PID），车上搭载可俯仰摆杆，钢球置于摆杆凹槽内；
视觉端（K230）通过 YOLO11 检测钢球位置，经 UART 发送给电控端（MSPM0G3507），
电控端以 PID 控制摆杆舵机/步进电机实现钢球平衡，OLED 实时显示任务状态与参数。

## 系统架构

```
┌─────────────────┐         UART (115200)          ┌──────────────────────┐
│  K230 视觉端     │  ────  $len,14,x,y,w,h,ball# ──▶ │  MSPM0G3507 电控端   │
│  YOLO11 钢球检测 │                                │  红外巡线 PID          │
│  卡尔曼滤波      │                                │  摆杆角度 PID          │
│  RTSP 无线图传   │                                │  电机驱动 + OLED 显示  │
└─────────────────┘                                └──────────────────────┘
```

## 目录结构

```
diansai-H-ball-balance/
├── README.md              # 本说明
├── firmware/              # MSPM0G3507 电控端（CCS + SysConfig 工程）
│   ├── main.c             # 主程序：菜单 + 任务状态机
│   ├── disansai.syscfg    # SysConfig 引脚/外设配置
│   ├── source/            # 模块驱动
│   │   ├── 0.96oled.c/h   # SSD1306 OLED (I2C)
│   │   ├── menu.c/h       # 菜单状态机（任务选择/参数查看/参数调节）
│   │   ├── key.c/h        # 4 键驱动（UP/DOWN/OK/BACK）
│   │   ├── ball_uart.c/h  # K230 球位接收 (UART1 中断)
│   │   ├── gyro.c/h       # JY-901B 陀螺仪 (UART2)
│   │   ├── balance.c/h    # 摆杆平衡控制
│   │   ├── line_follow.c  # 红外巡线
│   │   ├── three_ir_sensor.c/h # 三路红外
│   │   ├── ir_sensor.c/h  # 单路红外
│   │   ├── motor.c/h      # 直流电机 PWM 驱动
│   │   ├── y42_stepper.c/h # ZDT-Y42 闭环步进电机（摆杆）
│   │   ├── params.c/h     # PID 参数存储/调节
│   │   ├── timer_ms.c/h   # 毫秒计时
│   │   ├── delay.c/h      # 延时
│   │   └── uart.h / slave_protocol.h / car_control.h
│   ├── task/              # 任务状态机（T2~T6）
│   │   ├── task2.c/h      # 巡线一圈计时
│   │   ├── task3.c/h      # 静止摆球（中心↔±5cm）
│   │   ├── task4.c/h      # 行驶中摆球
│   │   ├── task5.c/h      # 行驶摆球 + 停止
│   │   └── task6.c/h
│   └── targetConfigs/     # CCS 调试配置
├── vision/                # K230 视觉端（CanMV K230 MicroPython）
│   ├── demo_rtsp.py       # 最终版：RTSP 图传 + YOLO11 + 卡尔曼 + UART
│   ├── demo-camera.py     # LCD 显示版：YOLO11 + 卡尔曼 + UART
│   ├── demo-picture.py    # 单张图片测试
│   ├── uart_send_test.py  # UART 发送联调测试
│   ├── yolo11n_det_320.kmodel # 钢球检测模型（320×320, conf=0.6）
│   ├── val.jpg            # 测试图片
│   └── dataset/           # 训练数据集（YOLO 格式 images/labels）
├── docs/                  # 项目文档
│   ├── H题_完整分工计划.md
│   ├── H题_视觉端完成总结.md
│   └── 技术报告模版.md
└── modules/               # 开发过程模块测试代码（供参考）
    ├── 10_DC_MOTOR_PID_2  # 直流电机 PID 测速
    ├── 11_BuJInDianJi     # 步进电机控制
    ├── SanLuHongWai(1)    # 三路红外巡线 v1
    └── SanLuHongWai(3)    # 三路红外巡线 v3
```

## 硬件接线

### K230 ↔ MSPM0（球位数据）

| K230 | MSPM0 | 功能 |
|------|-------|------|
| IO9 (TX) | PA18 (UART1 RX) | 球位置数据 |
| IO10 (RX) | PA17 (UART1 TX) | （可选） |
| GND | GND | 共地 |

### JY-901B 陀螺仪 ↔ MSPM0

| 陀螺仪 | MSPM0 | 功能 |
|--------|-------|------|
| TX | PA24 (UART2 RX) | 航向角 |
| RX | PA23 (UART2 TX) | 配置 |
| VCC | 3.3V / GND | 电源 |

### OLED 0.96" (SSD1306, I2C)

| 屏幕 | MSPM0 |
|------|-------|
| SDA | PB8 |
| SCL | PB9 |
| VCC/GND | 3.3V / GND |

## 编译与使用

### 电控端（firmware/）

1. 使用 **CCS (Code Composer Studio) 20.x + SysConfig** 打开工程。
2. 目标芯片：**MSPM0G3507**（天猛星/地猛星开发板）。
3. `disansai.syscfg` 为 SysConfig 配置文件，`Debug/` 目录由工程自动生成。
4. 编译下载后：OLED 进入菜单 → 选择任务 T2~T6 → 按键启动。

### 视觉端（vision/）

1. 将脚本与 `yolo11n_det_320.kmodel` 拷贝到 K230 SD 卡（模型放 `/sdcard/kmodel/`）。
2. `demo_rtsp.py`：修改 `WIFI_MODE` / SSID / 密码，运行后 VLC 打开
   `rtsp://<K230_IP>:8554/ball` 查看图传。
3. `demo-camera.py`：LCD 本地显示版，不依赖网络。
4. 标定：摆杆全长 250mm，画面 640px，默认 `MM_PER_PX = 250/640 ≈ 0.39mm/px`，
   现场可通过 `CALIB_X_MIN/MAX` 重新标定。

## 关键参数

| 参数 | 值 |
|------|-----|
| UART 波特率 | 115200 |
| 球位协议 | `$len,14,x,y,w,h,ball#` |
| YOLO 输入 | 320×320，conf=0.6，nms=0.45 |
| 卡尔曼 | Q=0.005, R=0.6, 掉帧上限 5 帧 |
| 巡线 PID | Kp=3.0, Kd=0.5 |
| 球控 PID | Kp=2.0, Kd=0.3 |
| 赛道黑线 | 宽 1.8±0.2cm |

## 说明

- 本仓库为竞赛开发全量代码整理，`modules/` 为开发过程中的模块测试工程，可直接复用。
- `docs/H题_视觉端完成总结.md` 记录了视觉端最终实现细节。
