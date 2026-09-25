# H题 视觉+显示端 完成情况

## 一、K230 视觉端

### 1. 钢球检测 (`demo_rtsp.py` / `demo-camera.py`)
- **YOLO11 目标检测**：320×320 模型，置信度 0.6
- **卡尔曼滤波**：位置平滑 + 掉帧预测（5帧内填补）
- **ROI 水平带过滤**：只识别中央区域的钢球，减少误检
- **尺寸/宽高比过滤**：10~50px，宽高比 0.65~1.55
- **自适应选择**：多个检测时选距预测位置最近的

### 2. 坐标计算
- 中心点：画面中央 X=320, Y=240
- 像素→毫米：0.39mm/px（摆杆 250mm / 640px）
- 输出偏差：`err_mm = (ball_x - 320) × 0.39`

### 3. RTSP 无线图传 (`demo_rtsp.py`)
- K230 开 AP 热点，笔记本直连
- 推流地址：`rtsp://192.168.169.1:8554/ball`
- 画面叠加 OSD：中心十字、ROI 框、球位偏差
- 码率 2000kbps，清晰流畅

### 4. UART 发送协议
- 格式：`$len,14,x,y,w,h,ball#`
- 波特率：115200
- 引脚：K230 IO9(TX) → MSPM0 PA18(RX)，K230 IO10(RX) → MSPM0 PA17(TX)

---

## 二、MSPM0 端

### 1. 文件结构
```
code/
├── main.c              # 主程序
├── source/
│   ├── 0.96oled.c/h    # OLED 驱动 (SSD1306, I2C)
│   ├── menu.c/h        # 菜单状态机
│   ├── key.c/h         # 按键驱动 (4键)
│   ├── ball_uart.c/h   # K230 球位接收 (UART1 中断)
│   ├── gyro.c/h        # 陀螺仪接收 (UART2 中断)
│   └── delay.h         # 延时函数
└── Debug/
    └── ti_msp_dl_config.c/h  # syscfg 生成（不要手动改）
```

### 2. 引脚分配
| MSPM0 | 功能 |
|-------|------|
| PA0 | 按键 UP |
| PA31 | 按键 DOWN |
| PA28 | 按键 OK |
| PA1 | 按键 BACK |
| PA18 | K230 UART RX |
| PA17 | K230 UART TX |
| PB8 | OLED SDA (I2C) |
| PB9 | OLED SCL (I2C) |
| PA24 | 陀螺仪 UART RX |
| PA23 | 陀螺仪 UART TX |
| PA10 | 调试串口 TX |

### 3. OLED 菜单结构
```
MAIN MENU
├── TASK SELECT → 选任务 T1~T6 → RUNNING 执行
├── BASIC TASKS → 同上
├── PARAM VIEW → 航向角 + 球位 + PID 参数
└── PARAM ADJ → 调节 PID 参数
```

按键：UP/DOWN=移动光标，OK=确认，BACK=返回。选中行白底黑字。

### 4. PARAM VIEW 显示数据
| 显示 | 含义 | 来源 |
|------|------|------|
| Y:xxx.x | 航向角 (度) | JY-901B 陀螺仪 |
| B:X=xxx E=xxmm | 球中心X + 水平偏差 | K230 视觉 |
| LKP/LKD | 巡线 PID | 本地参数 |
| BKP/BKD | 球控 PID | 本地参数 |

### 5. PID 参数默认值
| 参数 | 值 | 说明 |
|------|-----|------|
| LKP | 3.000 | 巡线 Kp |
| LKD | 0.500 | 巡线 Kd |
| BKP | 2.000 | 球控 Kp |
| BKD | 0.300 | 球控 Kd |

---

## 三、接线速查

```
K230                    MSPM0
IO9 (TX) ------------→  PA18 (UART1 RX)
IO10 (RX)------------←  PA17 (UART1 TX)
GND ------------------- GND

JY-901B 陀螺仪         MSPM0
TX ------------------→ PA24 (UART2 RX)
RX ------------------← PA23 (UART2 TX)
VCC ------------------ 3.3V
GND ------------------ GND

OLED 0.96"             MSPM0
SDA ------------------ PB8
SCL ------------------ PB9
VCC ------------------ 3.3V
GND ------------------ GND
```

## 四、待队友完成
- 硬件：小车底盘、摆杆机构、巡线红外、电机驱动
- 电控：PID 算法实现、舵机控制、巡线逻辑、任务状态机
- 联调：K230 上电自动运行、球位数据接入 PID
