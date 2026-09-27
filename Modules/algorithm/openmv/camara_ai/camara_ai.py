# -*- coding: utf-8 -*-
"""
camara_ai.py —— OpenMV 绿光识别模块（增强/Hybrid 版）
================================================================================
作用：
    把摄像头拍到的画面运算成“绿光目标”的中心坐标与尺寸 (x, y, w, h)。

本版 = 我们的封装骨架 + 深大开源代码里验证过的若干做法：
    1. 动态 ROI 跟踪：识别到目标后把搜索区收缩到光斑邻域(±FOUND_RANGE)，
       丢失则回到全图 INIT_ROI —— 更快、更抗噪（借鉴深大 build_found_roi）；
    2. 固定短曝光 + 固定增益：set_auto_exposure(False, exposure_us=800) 等，
       防止高速光点的运动模糊（借鉴深大 EXPOSURE_US=800 / gain_db=20）；
    3. 可选硬件读出窗 IOCTL_SET_READOUT_WINDOW：从传感器层面裁行，进一步提速；
    4. 帧超时容错 get_frame()：snapshot 异常重试 → 重初始化传感器（借鉴深大）；
    5. LED 指示：检测到亮绿灯、运行心跳蓝灯、开机闪烁（借鉴深大）。

对外接口：
    det = GreenLight()
    det.init_sensor()                     # 初始化(含开机 LED 指示)
    x, y, w, h, found = det.detect()      # 采一帧并检测
    x, y, w, h, found = det.detect(img)   # 传入已有帧

注意：
    * 绿色 LAB 阈值 GREEN_LAB 必须现场标定（默认取深大联调值，仅供参考）。
    * 坐标为像素，范围 QVGA: x∈[0,319], y∈[0,239]（默认）。
================================================================================
"""

import sensor
import image
import time

try:
    from pyb import LED
    _HAS_LED = True
except Exception:          # 主机端无 pyb，便于语法/逻辑自测
    _HAS_LED = False


# ============================================================================
#                              可调参数区
# ============================================================================

# 绿色 LAB 阈值（深大联调值，务必现场用阈值工具标定）
GREEN_LAB = (9, 100, -102, -25, 0, 127)

# 传感器参数
FRAMESIZE = sensor.QVGA
PIXFORMAT = sensor.RGB565
EXPOSURE_US = 3000         # 固定曝光, 防运动模糊; 远距离小光点 3000~4000, 近距离可降 800
GAIN_DB = 20               # 固定增益
SKIP_FRAMES_MS = 1500      # 初始化后跳帧，等图像稳定

# 硬件读出窗(可选)：如 (0, 700, 640, 480) 只读出传感器中间一片，提速/限视场；None 关闭
READOUT_WINDOW = None

# 动态 ROI 跟踪
INIT_ROI = (0, 0, 320, 240)  # 丢失时的全图搜索区
FOUND_RANGE_X = 40           # 命中后向左右各扩展的像素
FOUND_RANGE_Y = 40           # 命中后向上下各扩展的像素

# 色块筛选：远距离小光点只有 2~6 像素, 下限设小, 靠颜色/ROI 抑噪
PIXELS_THRESHOLD = 2
AREA_THRESHOLD = 2
MERGE = True
MERGE_MARGIN = 10
X_STRIDE = 1
Y_STRIDE = 1

# 坐标平滑与丢帧保持
SMOOTH_ALPHA = 0.4
LOST_HOLD_FRAMES = 5

# 调用方需告知图像尺寸用于 ROI 裁剪(默认 QVGA)
IMG_WIDTH = 320
IMG_HEIGHT = 240

# LED
USE_LED = True


# ============================================================================
#                              绿光检测类
# ============================================================================
class GreenLight(object):
    """绿光目标检测器（增强版：动态 ROI + 固定曝光 + 帧容错 + LED）。"""

    def __init__(self,
                 green_lab=GREEN_LAB,
                 init_roi=INIT_ROI,
                 pixels_threshold=PIXELS_THRESHOLD,
                 area_threshold=AREA_THRESHOLD,
                 smooth_alpha=SMOOTH_ALPHA,
                 lost_hold_frames=LOST_HOLD_FRAMES,
                 img_width=IMG_WIDTH,
                 img_height=IMG_HEIGHT):
        # 配置
        self.green_lab = green_lab
        self.init_roi = init_roi
        self.pixels_threshold = pixels_threshold
        self.area_threshold = area_threshold
        self.smooth_alpha = smooth_alpha
        self.lost_hold_frames = lost_hold_frames
        self.img_width = img_width
        self.img_height = img_height

        # 运行时状态
        self.cur_roi = init_roi      # 当前搜索区(动态收缩)
        self._fx = None              # 平滑后的 x
        self._fy = None              # 平滑后的 y
        self._fw = 0                 # 上一次宽
        self._fh = 0                 # 上一次高
        self._lost_cnt = 0           # 连续丢失计数
        self._sensor_ready = False

        # LED
        self._led_det = None
        if _HAS_LED and USE_LED:
            try:
                self._led_det = LED("LED_GREEN")
            except Exception:
                self._led_det = None

    # ------------------------------------------------------------------ #
    # 摄像头初始化
    # ------------------------------------------------------------------ #
    def init_sensor(self):
        """初始化 OpenMV 摄像头，并做开机 LED 指示(只调用一次)。"""
        # 1) 复位并设置格式/分辨率
        sensor.reset()
        sensor.set_framesize(FRAMESIZE)
        sensor.set_pixformat(PIXFORMAT)

        # 2) 可选：硬件读出窗裁剪(提速/限视场)
        if READOUT_WINDOW is not None:
            try:
                sensor.ioctl(sensor.IOCTL_SET_READOUT_WINDOW, READOUT_WINDOW)
            except Exception:
                pass

        # 3) 固定增益/白平衡/曝光(短曝光防糊)
        try:
            sensor.set_auto_gain(False, gain_db=GAIN_DB)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
        except Exception:
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_exposure(False)

        # 4) 跳帧等稳定
        try:
            sensor.skip_frames(time=SKIP_FRAMES_MS)
        except Exception:
            pass

        self._sensor_ready = True

        # 5) 开机 LED：绿灯闪 3 次表示启动成功
        if self._led_det is not None:
            for _ in range(3):
                self._led_det.on()
                time.sleep_ms(120)
                self._led_det.off()
                time.sleep_ms(120)

    # ------------------------------------------------------------------ #
    # 取帧(带容错)
    # ------------------------------------------------------------------ #
    def get_frame(self):
        """取一帧；偶发帧超时自动重试，连续失败则重初始化传感器。"""
        for _ in range(5):
            try:
                return sensor.snapshot()
            except Exception:
                time.sleep_ms(50)
        # 连续失败 → 重初始化后重试
        self.init_sensor()
        return sensor.snapshot()

    # ------------------------------------------------------------------ #
    # ROI 工具
    # ------------------------------------------------------------------ #
    def _clamp_roi(self, roi):
        """把 ROI 裁剪到图像范围，避免负坐标/越界。"""
        x, y, w, h = roi
        if x < 0:
            w += x
            x = 0
        if y < 0:
            h += y
            y = 0
        if x + w > self.img_width:
            w = self.img_width - x
        if y + h > self.img_height:
            h = self.img_height - y
        if w < 1:
            w = 1
        if h < 1:
            h = 1
        return (x, y, w, h)

    def build_found_roi(self, blob):
        """以光斑为中心向外扩 FOUND_RANGE，得到下一帧的贴身搜索区。"""
        return self._clamp_roi((
            blob.x() - FOUND_RANGE_X,
            blob.y() - FOUND_RANGE_Y,
            2 * FOUND_RANGE_X + blob.w(),
            2 * FOUND_RANGE_Y + blob.h(),
        ))

    # ------------------------------------------------------------------ #
    # 单帧检测
    # ------------------------------------------------------------------ #
    def detect(self, img=None):
        """采一帧(或使用传入帧)并检测绿光，返回 (x, y, w, h, found)。"""
        # 1) 取帧
        if img is None:
            img = self.get_frame()

        # 2) 在动态 ROI 内找绿色色块
        blobs = img.find_blobs(
            [self.green_lab],
            roi=self.cur_roi,
            x_stride=X_STRIDE,
            y_stride=Y_STRIDE,
            pixels_threshold=self.pixels_threshold,
            area_threshold=self.area_threshold,
            merge=MERGE,
            margin=MERGE_MARGIN,
        )

        # 3) 丢失：丢帧保持若干帧，否则回全图并报丢失
        if not blobs:
            self._lost_cnt += 1
            self.cur_roi = self.init_roi
            if self._led_det is not None:
                self._led_det.off()
            if self._lost_cnt > self.lost_hold_frames or self._fx is None:
                return 0, 0, 0, 0, False
            return int(self._fx), int(self._fy), self._fw, self._fh, True

        # 4) 命中：取像素数最大的块，收缩 ROI
        blob = max(blobs, key=lambda b: b.pixels())
        self._lost_cnt = 0
        self._fw, self._fh = blob.w(), blob.h()
        self.cur_roi = self.build_found_roi(blob)
        if self._led_det is not None:
            self._led_det.on()

        # 5) EMA 平滑
        cx, cy = blob.cxf(), blob.cyf()
        if self._fx is None:
            self._fx, self._fy = cx, cy
        else:
            a = self.smooth_alpha
            self._fx = a * cx + (1.0 - a) * self._fx
            self._fy = a * cy + (1.0 - a) * self._fy

        return int(self._fx), int(self._fy), self._fw, self._fh, True

    # ------------------------------------------------------------------ #
    # 调试接口
    # ------------------------------------------------------------------ #
    def find_best_blob(self, img):
        """返回当前帧面积最大的绿光色块对象(可能为 None)，便于画框/调试。"""
        blobs = img.find_blobs([self.green_lab], roi=self.cur_roi,
                               pixels_threshold=self.pixels_threshold,
                               area_threshold=self.area_threshold,
                               merge=MERGE, margin=MERGE_MARGIN)
        if not blobs:
            return None
        return max(blobs, key=lambda b: b.pixels())

    def reset(self):
        """清空平滑/丢失状态与 ROI(重新捕获目标时调用)。"""
        self._fx = None
        self._fy = None
        self._fw = 0
        self._fh = 0
        self._lost_cnt = 0
        self.cur_roi = self.init_roi
