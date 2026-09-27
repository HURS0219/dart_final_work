# -*- coding: utf-8 -*-
"""
camara_ai.py —— OpenMV 绿光识别模块
================================================================================
作用：
    把摄像头拍到的画面运算成“绿光目标”的坐标 (x, y)，供制导/自瞄使用。
    目标靶是一个绿光点，本模块负责“稳定、高效”地识别它并给出像素坐标。

设计要点（对应需求“保证稳定高效地识别”）：
    1. 只在 RGB565 彩色域下用 LAB 阈值找绿色，避免灰度丢失颜色信息；
    2. 用 find_blobs 做色块检测，取“面积最大的色块”作为绿光中心；
    3. 关闭自动增益/白平衡/曝光，保证同一阈值在不同时刻表现一致；
    4. 对坐标做一阶低通(EMA)平滑，抑制像素级抖动；
    5. 丢帧保持(lost_hold)：连续丢失若干帧才判定“目标丢失”，避免忽有忽无。

对外接口：
    det = GreenLight()            # 构造，可选传入自定义参数
    det.init_sensor()             # 初始化摄像头(只需一次)
    x, y, found = det.detect()    # 采一帧并检测，返回坐标与“是否找到”
    x, y, found = det.detect(img) # 也可传入已有的一帧 image 对象

注意：
    * 绿色 LAB 阈值 GREEN_LAB 必须现场标定（用 OpenMV IDE 的阈值工具取一个
      只框住绿光、框内其它区域尽量少的阈值），默认值仅供参考。
    * 坐标单位为像素，范围为当前分辨率 (默认 QVGA: x∈[0,319], y∈[0,239])。
================================================================================
"""

import sensor
import image
import time


# ============================================================================
#                              可调参数区
# ============================================================================

# 绿色 LAB 阈值 (L_min, L_max, A_min, A_max, B_min, B_max)
# 说明：绿光在 LAB 中 A(绿-红) 分量明显偏负，B(蓝-黄) 依光源略有差异。
#      务必用 OpenMV IDE 阈值工具现场标定后替换本默认值。
GREEN_LAB = (20, 100, -80, -25, -30, 45)

# 检测感兴趣区域 ROI：None 表示整幅图；也可传 (x, y, w, h) 缩小搜索范围以提速
ROI = None

# 色块筛选：像素数/面积下限，滤掉噪点；merge 合并相邻色块，margin 为合并间距
PIXELS_THRESHOLD = 20
AREA_THRESHOLD = 20
MERGE = True
MERGE_MARGIN = 10

# 扫描步长：越大越快但越粗；绿光点较小时建议 1~2
X_STRIDE = 2
Y_STRIDE = 2

# 坐标平滑系数 (0~1)：越大越“跟手”，越小越平滑但延迟越大
SMOOTH_ALPHA = 0.4

# 丢帧保持帧数：连续丢失超过该帧数才判“目标丢失”，否则沿用上次坐标
LOST_HOLD_FRAMES = 5

# 摄像头初始化参数：跳帧数(等自动曝光稳定)与是否关闭自动调节
SKIP_FRAMES_MS = 2000
LOCK_AUTO = True      # True: 关闭自动增益/白平衡/曝光，保证阈值稳定
FRAMESIZE = sensor.QVGA
PIXFORMAT = sensor.RGB565


# ============================================================================
#                              绿光检测类
# ============================================================================
class GreenLight(object):
    """绿光目标检测器：封装摄像头初始化与坐标输出。"""

    def __init__(self,
                 green_lab=GREEN_LAB,
                 roi=ROI,
                 pixels_threshold=PIXELS_THRESHOLD,
                 area_threshold=AREA_THRESHOLD,
                 smooth_alpha=SMOOTH_ALPHA,
                 lost_hold_frames=LOST_HOLD_FRAMES):
        """构造函数：仅保存配置，不接触硬件，便于在主机端做参数检查。"""
        self.green_lab = green_lab
        self.roi = roi
        self.pixels_threshold = pixels_threshold
        self.area_threshold = area_threshold
        self.smooth_alpha = smooth_alpha
        self.lost_hold_frames = lost_hold_frames

        # 运行时状态
        self._fx = None            # 平滑后的 x
        self._fy = None            # 平滑后的 y
        self._lost_cnt = 0         # 连续丢失计数
        self._sensor_ready = False # 摄像头是否已初始化

    # ------------------------------------------------------------------ #
    # 摄像头初始化
    # ------------------------------------------------------------------ #
    def init_sensor(self):
        """初始化 OpenMV 摄像头：(只需在程序开始时调用一次)"""
        # 1) 复位并设置彩色格式与分辨率
        sensor.reset()
        sensor.set_pixformat(PIXFORMAT)
        sensor.set_framesize(FRAMESIZE)

        # 2) 跳过若干帧，等待自动曝光/白平衡收敛
        sensor.skip_frames(time=SKIP_FRAMES_MS)

        # 3) 锁死自动调节，保证不同时刻的成像一致（阈值才能稳定命中）
        if LOCK_AUTO:
            sensor.set_auto_gain(False)        # 关闭自动增益
            sensor.set_auto_whitebal(False)    # 关闭自动白平衡
            sensor.set_auto_exposure(False)    # 关闭自动曝光
        self._sensor_ready = True

    # ------------------------------------------------------------------ #
    # 单帧检测
    # ------------------------------------------------------------------ #
    def detect(self, img=None):
        """采一帧(或使用传入帧)并检测绿光。

        参数:
            img: 可选，已有的 image 对象；为 None 时内部 snapshot 取一帧。
        返回:
            (x, y, found)
            x, y : 像素坐标(整数)
            found: True=本帧找到目标；False=丢帧保持也用尽，判为丢失
        """
        # 1) 取帧
        if img is None:
            img = sensor.snapshot()

        # 2) 色块检测：只保留绿色、面积足够大的色块，必要时合并相邻块
        blobs = img.find_blobs(
            [self.green_lab],          # 阈值列表（可放多个阈值）
            roi=self.roi,              # 搜索区域
            x_stride=X_STRIDE,         # 横向扫描步长(提速)
            y_stride=Y_STRIDE,         # 纵向扫描步长(提速)
            pixels_threshold=self.pixels_threshold,  # 像素数下限
            area_threshold=self.area_threshold,      # 外接矩形面积下限
            merge=MERGE,               # 合并相邻色块
            margin=MERGE_MARGIN,       # 合并间距
        )

        # 3) 没找到：进入“丢帧保持”，连续丢失超过阈值才判丢失
        if not blobs:
            self._lost_cnt += 1
            if self._lost_cnt > self.lost_hold_frames or self._fx is None:
                return 0, 0, False
            return int(self._fx), int(self._fy), True

        # 4) 找到：取面积(像素数)最大的色块作为绿光
        blob = max(blobs, key=lambda b: b.pixels())
        self._lost_cnt = 0

        # 5) 坐标平滑(一阶低通 EMA)：首帧直接采用，之后按 alpha 融合
        cx, cy = blob.cxf(), blob.cyf()
        if self._fx is None:
            self._fx, self._fy = cx, cy
        else:
            a = self.smooth_alpha
            self._fx = a * cx + (1.0 - a) * self._fx
            self._fy = a * cy + (1.0 - a) * self._fy

        # 6) 返回整数像素坐标
        return int(self._fx), int(self._fy), True

    # ------------------------------------------------------------------ #
    # 便捷接口：取原始(未平滑)色块，供调试/画框使用
    # ------------------------------------------------------------------ #
    def find_best_blob(self, img):
        """返回当前帧面积最大的绿光色块对象(可能为 None)，便于画框/调试。"""
        blobs = img.find_blobs([self.green_lab], roi=self.roi,
                               pixels_threshold=self.pixels_threshold,
                               area_threshold=self.area_threshold,
                               merge=MERGE, margin=MERGE_MARGIN)
        if not blobs:
            return None
        return max(blobs, key=lambda b: b.pixels())

    def reset(self):
        """清空平滑状态与丢失计数(重新捕获目标时调用)。"""
        self._fx = None
        self._fy = None
        self._lost_cnt = 0
