"""
YOLO11 钢球检测 + 卡尔曼滤波
ROI/尺寸/宽高比过滤 + 预测填掉帧
实验平台：01Studio CanMV K230
"""

from libs.PipeLine import PipeLine
from libs.YOLO import YOLO11
from libs.Utils import *
from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart
from media.sensor import *
import os, sys, gc
import ulab.numpy as np
import image, time, math

# ============================================================
# 配置
# ============================================================

kmodel_path = "/sdcard/kmodel/yolo11n_det_320.kmodel"
labels = {0: '0'}
model_input_size = [320, 320]
display = "lcd3_5"

if display == "hdmi":
    display_mode = "hdmi"
    display_size = [1920, 1080]
elif display == "lcd2_4":
    display_mode = "st7701"
    display_size = [640, 480]
else:
    display_mode = "lcd"
    display_size = [640, 480]

rgb888p_size = [640, 480]

# ---- 标定参数 (现场实测后修改) ----
# 方法: 球放摆杆最左端看终端[PRED]行X值→填CALIB_X_MIN
#       球放摆杆最右端看终端[PRED]行X值→填CALIB_X_MAX
#       MM_PER_PX = 250/(X_MAX-X_MIN), CENTER = (X_MIN+X_MAX)/2
CALIB_X_MIN = 0      # 摆杆0mm处像素X (实测后修改)
CALIB_X_MAX = 640    # 摆杆250mm处像素X (实测后修改)
CALIB_ENABLE = False  # True=启用标定

if CALIB_ENABLE and CALIB_X_MAX > CALIB_X_MIN:
    CENTER_X = (CALIB_X_MIN + CALIB_X_MAX) // 2
    MM_PER_PX = 250.0 / (CALIB_X_MAX - CALIB_X_MIN)
else:
    CENTER_X = rgb888p_size[0] // 2   # 320
    MM_PER_PX = 250.0 / 640.0          # 0.39 mm/px

CENTER_Y = (240 + 300) // 2            # 270 (新ROI中心)

# ROI 水平带下半部分: Y ∈ [240, 300]
ROI_TOP    = rgb888p_size[1] // 2       # 240
ROI_BOTTOM = rgb888p_size[1] // 2 + rgb888p_size[1] // 8   # 300
BALL_MIN_W, BALL_MAX_W = 10, 50
BALL_MIN_H, BALL_MAX_H = 10, 50
ASPECT_MIN, ASPECT_MAX = 0.65, 1.55

# ---- 卡尔曼参数 ----
KALMAN_Q = 0.005
KALMAN_R = 0.6
MAX_DROPOUT = 5   # 连续掉帧上限

# ---- YOLO ----
confidence_threshold = 0.6
nms_threshold = 0.45

# ============================================================
# 卡尔曼滤波器
# ============================================================

class KalmanTracker:
    def __init__(self, q=0.005, r=0.6):
        self.x = 0.0; self.v = 0.0
        self.p_xx = 100.0; self.p_xv = 0.0; self.p_vv = 100.0
        self.q = q; self.r = r
        self.inited = False

    def update(self, z):
        if not self.inited:
            self.x = float(z); self.v = 0.0; self.inited = True
            return self.x
        xp = self.x + self.v
        pxx_p = self.p_xx + 2.0 * self.p_xv + self.p_vv + self.q
        pxv_p = self.p_xv + self.p_vv
        pvv_p = self.p_vv + self.q
        s = pxx_p + self.r
        kx = pxx_p / s; kv = pxv_p / s
        y = z - xp
        self.x = xp + kx * y
        self.v = self.v + kv * y
        self.p_xx = pxx_p - kx * pxx_p
        self.p_xv = pxv_p - kx * pxv_p
        self.p_vv = pvv_p - kv * pxv_p
        return self.x

    def predict(self):
        if not self.inited: return self.x
        self.x += self.v
        self.p_xx = self.p_xx + 2.0 * self.p_xv + self.p_vv + self.q
        self.p_xv = self.p_xv + self.p_vv
        self.p_vv = self.p_vv + self.q
        return self.x

# ============================================================
# 初始化
# ============================================================

kx = KalmanTracker(KALMAN_Q, KALMAN_R)
ky = KalmanTracker(KALMAN_Q, KALMAN_R)

pl = PipeLine(rgb888p_size=rgb888p_size, display_size=display_size, display_mode=display_mode)
if display == "lcd2_4":
    pl.create(sensor=Sensor(id=2, width=1280, height=960))
else:
    pl.create(sensor=Sensor(id=2, width=1920, height=1080))

display_size = pl.get_display_size()

yolo = YOLO11(
    task_type="detect", mode="video",
    kmodel_path=kmodel_path, labels=labels,
    rgb888p_size=rgb888p_size, model_input_size=model_input_size,
    display_size=display_size,
    conf_thresh=confidence_threshold, nms_thresh=nms_threshold,
    max_boxes_num=50, debug_mode=0,
)
yolo.config_preprocess()

uart = YbUart(baudrate=115200)
pto  = YbProtocol()

# ============================================================
# 主循环
# ============================================================

clock = time.clock()
dropouts = 0

print("=" * 50)
print("YOLO+Kalman 钢球追踪")
print(f"ROI: y=[{ROI_TOP},{ROI_BOTTOM}] CENTER={CENTER_X} {MM_PER_PX:.3f}mm/px")
print("=" * 50)

while True:
    clock.tick()

    img = pl.get_frame()
    res = yolo.run(img)

    # ---- 筛选最佳钢球检测 ----
    best_det = None  # (cx, cy, x, y, w, h, label)

    if res and len(res[0]) > 0:
        for i in range(len(res[0])):
            x, y, w, h = map(lambda v: int(round(v, 0)), res[0][i])
            label = labels.get(res[1][i], '?')
            cx = x + w // 2
            cy = y + h // 2

            # ROI 水平带
            if not (ROI_TOP <= cy <= ROI_BOTTOM):
                continue
            # 尺寸
            if not (BALL_MIN_W <= w <= BALL_MAX_W and BALL_MIN_H <= h <= BALL_MAX_H):
                continue
            # 宽高比
            ratio = w / h
            if not (ASPECT_MIN <= ratio <= ASPECT_MAX):
                continue

            # 选最佳：距卡尔曼预测位置最近的
            if kx.inited:
                pred_x = kx.x + kx.v
                pred_y = ky.x + ky.v
                dist = math.sqrt((cx - pred_x) ** 2 + (cy - pred_y) ** 2)
                if best_det is None or dist < best_det[0]:
                    best_det = (dist, cx, cy, x, y, w, h, label)
            else:
                # 首帧：选置信度最高的 (res[2][i])
                conf = res[2][i] if len(res) > 2 and i < len(res[2]) else 0.5
                if best_det is None or conf > best_det[0]:
                    best_det = (conf, cx, cy, x, y, w, h, label)

    # ---- 显示 ----
    pl.osd_img.clear()

    # 画 ROI 水平带 + 中心十字
    pl.osd_img.draw_rectangle(0, ROI_TOP, 640, ROI_BOTTOM - ROI_TOP, color=(80, 80, 80), thickness=1)
    pl.osd_img.draw_line(0, CENTER_Y, 640, CENTER_Y, color=(60, 60, 60), thickness=1)
    pl.osd_img.draw_line(CENTER_X, 0, CENTER_X, 480, color=(60, 60, 60), thickness=1)
    pl.osd_img.draw_cross(CENTER_X, CENTER_Y, color=(255, 0, 0), size=12, thickness=2)

    # ---- 卡尔曼 + UART ----
    if best_det:
        _, cx, cy, x, y, w, h, label = best_det
        dropouts = 0
        kx.update(cx)
        ky.update(cy)

        # 发送预测位置: 当前+速度×3帧 (补偿~100ms延迟)
        px = int(kx.x + kx.v * 3)
        py = int(ky.x + ky.v * 3)
        data = pto.get_object_detect_data(px-10, py-10, 20, 20, label)
        uart.send(data)
        print(f"[BALL] ({int(kx.x)},{int(ky.x)}) err={(int(kx.x)-CENTER_X)*MM_PER_PX:+.1f}mm")

    elif dropouts < MAX_DROPOUT:
        dropouts += 1
        kx.predict(); ky.predict()
        px = int(kx.x + kx.v * 3)
        py = int(ky.x + ky.v * 3)
        data = pto.get_object_detect_data(px-10, py-10, 20, 20, "ball")
        uart.send(data)
        print(f"[PRED] #{dropouts}")

    else:
        print(f"[LOST] {dropouts}帧")

    # ---- 画OSD (不分检测/预测, 只要卡尔曼有值就画) ----
    if kx.inited:
        ix, iy = int(kx.x), int(ky.x)
        err_mm = (ix - CENTER_X) * MM_PER_PX

        draw_color = (0, 255, 0) if dropouts < 3 else (255, 255, 0)

        pl.osd_img.draw_cross(ix, iy, color=draw_color, size=10, thickness=2)
        pl.osd_img.draw_line(CENTER_X, CENTER_Y, ix, iy, color=(0, 200, 255), thickness=1)
        pl.osd_img.draw_string_advanced(ix+14, iy-6, 16, f"d={err_mm:+.0f}mm", color=(0, 200, 255))
        if best_det:
            x, y, w, h = best_det[3], best_det[4], best_det[5], best_det[6]
            pl.osd_img.draw_rectangle(x, y, w, h, color=(0, 255, 0), thickness=2)

    # ---- FPS ----
    pl.osd_img.draw_string_advanced(4, 4, 16, f"FPS:{clock.fps():.0f}", color=(255, 255, 255))
    pl.osd_img.draw_string_advanced(4, 20, 16, "YOLO+KALMAN", color=(0, 255, 0))

    pl.show_image()
    gc.collect()

yolo.deinit()
pl.destroy()
