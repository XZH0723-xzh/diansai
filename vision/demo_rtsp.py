"""
RTSP推流 + YOLO11检测 + 卡尔曼滤波 + UART
双通道sensor: CHN0=YUV推流 CHN1=RGB给AI
实验平台：01Studio CanMV K230
"""

import network, os, time, _thread, gc, sys, math
import ujson, utime, uctypes
import ulab.numpy as np
import nncase_runtime as nn
import aidemo, image
import multimedia as mm
from time import sleep
from media.vencoder import *
from media.sensor import *
from media.media import *
from media.display import *
from libs.PipeLine import PipeLine, ScopedTiming
from libs.AIBase import AIBase
from libs.AI2D import Ai2d
from libs.YOLO import YOLO11
from libs.YbProtocol import YbProtocol
from ybUtils.YbUart import YbUart

# ============================================================
# 参数
# ============================================================

# WiFi 模式: "ap"=K230开热点(赛场用), "sta"=连路由器(调试用)
WIFI_MODE = "sta"   # K230直连笔记本热点

# AP模式 (K230自己开热点, 笔记本直连) —— 自行修改为你想要的热点名/密码
AP_SSID = "K230_Ball"
AP_PASS = "12345678"

# STA模式 (K230连外部WiFi) —— 填写你自己的 WiFi 账号密码
STA_SSID = "your_wifi_ssid"
STA_PASS = "your_wifi_password"

# 帧尺寸
AI_W, AI_H = 640, 480       # AI检测分辨率
RTSP_W, RTSP_H = 640, 480   # 推流分辨率

# YOLO
KMODEL_PATH = "/sdcard/kmodel/yolo11n_det_320.kmodel"
LABELS = {0: '0'}
MODEL_INPUT = [320, 320]
CONF_THRESH = 0.6
NMS_THRESH = 0.45

# 中心点 + 毫米映射
CENTER_X = AI_W // 2    # 320
CENTER_Y = AI_H // 2    # 240
MM_PER_PX = 250.0 / 640.0  # 摆管250mm / 画面640px

# ROI 水平带
ROI_TOP    = AI_H // 2 - AI_H // 8   # 180
ROI_BOTTOM = AI_H // 2 + AI_H // 8   # 300

# 过滤
BALL_MIN_W, BALL_MAX_W = 10, 50
BALL_MIN_H, BALL_MAX_H = 10, 50
ASPECT_MIN, ASPECT_MAX = 0.65, 1.55

# 卡尔曼
KALMAN_Q = 0.005
KALMAN_R = 0.6
MAX_DROPOUT = 5

# ============================================================
# WiFi
# ============================================================

def wifi_setup():
    if WIFI_MODE == "ap":
        # K230开热点 (赛场用，笔记本直连)
        ap = network.WLAN(network.AP_IF)
        if not ap.active():
            ap.active(True)
        print("[WIFI] AP activated:", ap.active())
        ap.config(ssid=AP_SSID, key=AP_PASS)
        print(f"[WIFI] SSID={AP_SSID} key={AP_PASS}")
        time.sleep(2)
        ip = ap.ifconfig()[0]
        print(f"[WIFI] AP IP={ip}")
        print(f"[WIFI] 笔记本连 {AP_SSID}, VLC: rtsp://{ip}:8554/ball")
        return ip
    else:
        # K230连路由器 (调试用)
        sta = network.WLAN(0)
        sta.connect(STA_SSID, STA_PASS)
        while sta.ifconfig()[0] == '0.0.0.0':
            time.sleep(1)
        ip = sta.ifconfig()[0]
        print(f"[WIFI] STA模式 IP={ip}")
        return ip

# ============================================================
# 卡尔曼
# ============================================================

class KalmanTracker:
    def __init__(self, q=0.005, r=0.6):
        self.x=0.0; self.v=0.0
        self.p_xx=100.0; self.p_xv=0.0; self.p_vv=100.0
        self.q=q; self.r=r; self.inited=False

    def update(self, z):
        if not self.inited:
            self.x=float(z); self.v=0.0; self.inited=True; return self.x
        xp=self.x+self.v
        pxx_p=self.p_xx+2.0*self.p_xv+self.p_vv+self.q
        pxv_p=self.p_xv+self.p_vv
        pvv_p=self.p_vv+self.q
        s=pxx_p+self.r; kx=pxx_p/s; kv=pxv_p/s; y=z-xp
        self.x=xp+kx*y; self.v=self.v+kv*y
        self.p_xx=pxx_p-kx*pxx_p
        self.p_xv=pxv_p-kx*pxv_p
        self.p_vv=pvv_p-kv*pxv_p
        return self.x

    def predict(self):
        if not self.inited: return self.x
        self.x+=self.v
        self.p_xx=self.p_xx+2.0*self.p_xv+self.p_vv+self.q
        self.p_xv=self.p_xv+self.p_vv
        self.p_vv=self.p_vv+self.q
        return self.x

# ============================================================
# RTSP 服务器 (集成YOLO检测)
# ============================================================

class BallRtspServer:
    def __init__(self):
        self.session_name = "ball"
        self.port = 8554
        self.venc_chn = VENC_CHN_ID_0
        self.start_stream = False
        self.runthread_over = False
        self.sensor = None
        self.yolo = None
        self.uart = None
        self.pto = None
        self._last_det = None
        self.kx = KalmanTracker(KALMAN_Q, KALMAN_R)
        self.ky = KalmanTracker(KALMAN_Q, KALMAN_R)
        self.dropouts = 0

        # 初始化UART
        self.uart = YbUart(baudrate=115200)
        self.pto = YbProtocol()

    def start(self):
        self._init_sensor()
        self._init_yolo()
        self._init_encoder()
        self._init_rtsp()

    def _init_sensor(self):
        self.sensor = Sensor()
        self.sensor.reset()

        # CHN0: YUV420SP → 推流 + LCD显示
        self.sensor.set_framesize(width=RTSP_W, height=RTSP_H, alignment=12)
        self.sensor.set_pixformat(Sensor.YUV420SP)

        # CHN1: RGB888_PLANAR → AI检测
        self.sensor.set_framesize(width=AI_W, height=AI_H, chn=CAM_CHN_ID_1)
        self.sensor.set_pixformat(PIXEL_FORMAT_RGB_888_PLANAR, chn=CAM_CHN_ID_1)

        # RTSP走网络，LCD不需要（赛场笔记本VLC看）

    def _init_yolo(self):
        self.yolo = YOLO11(
            task_type="detect", mode="video",
            kmodel_path=KMODEL_PATH, labels=LABELS,
            rgb888p_size=[AI_W, AI_H],
            model_input_size=MODEL_INPUT,
            display_size=[AI_W, AI_H],
            conf_thresh=CONF_THRESH, nms_thresh=NMS_THRESH,
            max_boxes_num=50, debug_mode=0,
        )
        self.yolo.config_preprocess()

    def _init_encoder(self):
        w = ALIGN_UP(RTSP_W, 16)
        self.encoder = Encoder()
        self.encoder.SetOutBufs(self.venc_chn, 8, w, RTSP_H)
        MediaManager.init()
        chnAttr = ChnAttrStr(
            self.encoder.PAYLOAD_TYPE_H264,
            self.encoder.H264_PROFILE_MAIN,
            w, RTSP_H,
            bit_rate=2000, dst_frame_rate=25, src_frame_rate=25
        )
        self.encoder.Create(self.venc_chn, chnAttr)

    def _init_rtsp(self):
        self.rtspserver = mm.rtsp_server()
        self.rtspserver.rtspserver_init(self.port)
        self.rtspserver.rtspserver_createsession(
            self.session_name,
            mm.multi_media_type.media_h264,
            False
        )
        self.rtspserver.rtspserver_start()

        self.encoder.Start(self.venc_chn)
        self.sensor.run()

        self.start_stream = True
        _thread.start_new_thread(self._stream_loop, ())

    def get_url(self):
        return self.rtspserver.rtspserver_getrtspurl(self.session_name)

    def stop(self):
        if not self.start_stream: return
        self.start_stream = False
        while not self.runthread_over:
            sleep(0.1)
        self.runthread_over = False
        self.sensor.stop()
        self.encoder.Stop(self.venc_chn)
        self.encoder.Destroy(self.venc_chn)
        self.rtspserver.rtspserver_stop()
        self.rtspserver.rtspserver_deinit()
        MediaManager.deinit()

    # ---- 检测逻辑 ----

    def filter_detections(self, res):
        """从YOLO结果中筛选最佳钢球"""
        best = None  # (cx, cy, x, y, w, h, label)
        best_score = None

        if not res or len(res[0]) == 0:
            return None

        for i in range(len(res[0])):
            x, y, w, h = map(lambda v: int(round(v, 0)), res[0][i])
            label = LABELS.get(res[1][i], '?')
            cx = x + w // 2
            cy = y + h // 2

            if not (ROI_TOP <= cy <= ROI_BOTTOM): continue
            if not (BALL_MIN_W <= w <= BALL_MAX_W and BALL_MIN_H <= h <= BALL_MAX_H): continue
            ratio = w / h
            if not (ASPECT_MIN <= ratio <= ASPECT_MAX): continue

            # 评分：距卡尔曼预测越近越好
            if self.kx.inited:
                px = self.kx.x + self.kx.v
                py = self.ky.x + self.ky.v
                score = -math.sqrt((cx-px)**2 + (cy-py)**2)  # 负距离，越大越好
            else:
                conf = res[2][i] if len(res) > 2 and i < len(res[2]) else 0.5
                score = conf

            if best is None or (best_score is None or score > best_score):
                best_score = score
                best = (cx, cy, x, y, w, h, label)

        return best

    def draw_osd(self, img):
        """YUV OSD: ROI框 + 中心线 + 球位十字 + 偏差 + 检测框"""
        # ROI水平带 (空心框)
        img.draw_rectangle(0, ROI_TOP, AI_W, ROI_BOTTOM-ROI_TOP, color=(80,80,80), thickness=1)
        # 中心横线
        img.draw_rectangle(0, CENTER_Y, AI_W, 1, color=(80,80,80))
        # 中心十字
        img.draw_rectangle(CENTER_X-6, CENTER_Y, 12, 1)
        img.draw_rectangle(CENTER_X, CENTER_Y-6, 1, 12)

        if not self.kx.inited:
            return

        ix, iy = int(self.kx.x), int(self.ky.x)
        err_mm = (ix - CENTER_X) * MM_PER_PX
        color = (0, 255, 0) if self.dropouts < 3 else (255, 255, 0)

        # 检测框 (YOLO原始)
        if self._last_det:
            rx, ry, rw, rh = self._last_det[2], self._last_det[3], self._last_det[4], self._last_det[5]
            img.draw_rectangle(rx, ry, rw, rh, color=(0, 255, 0), thickness=2)
        # 球位十字
        img.draw_rectangle(ix-5, iy, 10, 1, color=color)
        img.draw_rectangle(ix, iy-5, 1, 10, color=color)
        # 偏差值
        img.draw_string(ix+14, iy-6, f"d={err_mm:+.0f}mm", color=(0,200,255))

    def _stream_loop(self):
        try:
            streamData = StreamData()
            frame_info = k_video_frame_info()

            while self.start_stream:
                # ---- AI: 取RGB帧做检测 ----
                ai_img = self.sensor.snapshot(chn=CAM_CHN_ID_1)
                if ai_img == -1: continue
                np_img = ai_img.to_numpy_ref()
                res = self.yolo.run(np_img)

                # ---- 筛选 + 卡尔曼 ----
                det = self.filter_detections(res)

                if det:
                    self._last_det = det
                    cx, cy, x, y, w, h, label = det
                    self.dropouts = 0
                    self.kx.update(cx); self.ky.update(cy)

                    ws = max(w,10); hs = max(h,10)
                    data = self.pto.get_object_detect_data(x, y, ws, hs, label)
                    self.uart.send(data)
                    print(f"[BALL] ({int(self.kx.x)},{int(self.ky.x)}) err={(int(self.kx.x)-CENTER_X)*MM_PER_PX:+.1f}mm")

                elif self.dropouts < MAX_DROPOUT:
                    self.dropouts += 1
                    self.kx.predict(); self.ky.predict()
                    data = self.pto.get_object_detect_data(int(self.kx.x)-10, int(self.ky.x)-10, 20, 20, "ball")
                    self.uart.send(data)
                    print(f"[PRED] #{self.dropouts}")

                else:
                    print(f"[LOST] {self.dropouts}帧")

                # ---- 推流: YUV帧+OSD+编码 ----
                yuv_img = self.sensor.snapshot(chn=CAM_CHN_ID_0)
                if yuv_img == -1: continue
                self.draw_osd(yuv_img)

                # ---- 编码推流 ----
                frame_info.v_frame.width = yuv_img.width()
                frame_info.v_frame.height = yuv_img.height()
                frame_info.v_frame.pixel_format = Sensor.YUV420SP
                frame_info.pool_id = yuv_img.poolid()
                frame_info.v_frame.phys_addr[0] = yuv_img.phyaddr()
                frame_info.v_frame.phys_addr[1] = frame_info.v_frame.phys_addr[0] + \
                    frame_info.v_frame.width * frame_info.v_frame.height

                self.encoder.SendFrame(self.venc_chn, frame_info)
                self.encoder.GetStream(self.venc_chn, streamData)

                for pack_idx in range(streamData.pack_cnt):
                    stream_data = bytes(uctypes.bytearray_at(
                        streamData.data[pack_idx], streamData.data_size[pack_idx]))
                    self.rtspserver.rtspserver_sendvideodata(
                        self.session_name, stream_data,
                        streamData.data_size[pack_idx], 1000)

                self.encoder.ReleaseStream(self.venc_chn, streamData)

                gc.collect()
                time.sleep_us(10)
                os.exitpoint()

        except BaseException as e:
            print(f"[RTSP ERR] {e}")
        finally:
            self.runthread_over = True
            self.stop()
        self.runthread_over = True

# ============================================================
# 主入口
# ============================================================

if __name__ == "__main__":
    print("[WIFI] Starting...")
    ip = wifi_setup()

    print("[RTSP] Starting...")
    server = BallRtspServer()
    server.start()
    url = server.get_url()
    print(f"[RTSP] {url}")
    print("=" * 50)
    print(f"VLC打开: {url}")
    print("=" * 50)

    # 主线程空闲，RTSP在后台线程运行
    while True:
        time.sleep(60)
