# -*- coding: utf-8 -*-
"""
openmv_stream.py —— OpenMV 端：把实时画面以 JPEG 通过 USB 串口推给 PC
================================================================================
配合 pc_view/openmv_viewer.py 在电脑上开窗口实时查看画面（方便调试阈值/曝光/瞄准）。

帧协议(与 viewer 约定)：
    [0xA5][0x5A] [len:4字节 小端] [JPEG 数据 len 字节]
    在 JPEG 前方加了固定 2 字节magic + 4 字节长度，便于 PC 端对齐/断帧恢复。

部署：用 raw REPL 把它写成 OpenMV 根目录的 main.py，或直接 mpremote run。
运行后若没有 PC 在读，USB 写会阻塞——这是正常的。
================================================================================
"""

import sensor
import image
import time
import struct
from pyb import USB_VCP

# ------------------------------ 可调参数 ------------------------------
# 实测(OpenMV H7, USB VCP)：QVGA q60+检测≈9.6fps；QVGA q50 不检测≈14.5fps；QQVGA q50≈19fps
JPEG_QUALITY = 50          # JPEG 质量(越小越流畅)
FRAMESIZE = sensor.QVGA    # 分辨率(改 QQVGA 更快)
TARGET_FPS = 30            # 目标帧率(0=不限)
DRAW_OVERLAY = True        # 是否画准星/FPS 文字
DETECT = True              # 是否顺带做绿光检测并画框
GREEN_LAB = (9, 100, -102, -25, 0, 127)  # 绿光阈值(现场标定; 实测远距离更灵敏)

# 曝光：False=固定短曝光(检测最优, 实测 100%命中/抖动9px; 画面会偏暗, 只有亮点)
#       True =自动曝光(画面亮但抖动大, 仅用于看环境)
AUTO_EXPOSURE = False
EXPOSURE_US = 3000        # 远距离小光点用 3000~4000; 近距离可降回 800
GAIN_DB = 20

MAGIC = b'\xA5\x5A'        # 与 PC 端一致的帧头
# ---------------------------------------------------------------------


def init_sensor():
    """初始化摄像头。默认固定短曝光(检测最优)；AUTO_EXPOSURE=True 时用自动曝光。"""
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(FRAMESIZE)
    if AUTO_EXPOSURE:
        sensor.set_auto_gain(True)
        sensor.set_auto_whitebal(True)
        try:
            sensor.set_auto_exposure(True)
        except Exception:
            pass
    else:
        try:
            sensor.set_auto_gain(False, gain_db=GAIN_DB)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)
        except Exception:
            sensor.set_auto_gain(False)
            sensor.set_auto_whitebal(False)
            sensor.set_auto_exposure(False)
    try:
        sensor.skip_frames(time=1500)
    except Exception:
        pass


def main():
    init_sensor()
    usb = USB_VCP()
    clock = time.clock()
    period_ms = int(1000 / TARGET_FPS) if TARGET_FPS > 0 else 0

    while True:
        t0 = time.ticks_ms()
        clock.tick()

        # 1) 取帧
        try:
            img = sensor.snapshot()
        except Exception:
            time.sleep_ms(30)
            continue

        # 2) 可选：绿光检测并画框(调试识别用)
        if DETECT:
            blobs = img.find_blobs([GREEN_LAB], pixels_threshold=2,
                                   area_threshold=2, merge=True, margin=10)
            if blobs:
                b = max(blobs, key=lambda x: x.pixels())
                img.draw_rectangle(b.rect(), color=(255, 0, 0))
                img.draw_cross(int(b.cxf()), int(b.cyf()), color=(0, 255, 0))

        # 3) 可选：准星 + FPS
        if DRAW_OVERLAY:
            img.draw_cross(img.width() // 2, img.height() // 2, color=(0, 128, 255))
            img.draw_string(2, 2, "FPS %.1f" % clock.fps(), color=(255, 255, 0))

        # 4) JPEG 压缩并加帧头推送
        jpg = img.compress(quality=JPEG_QUALITY)
        usb.write(MAGIC + struct.pack("<I", len(jpg)) + jpg)

        # 5) 限帧
        if period_ms:
            dt = time.ticks_diff(time.ticks_ms(), t0)
            if dt < period_ms:
                time.sleep_ms(period_ms - dt)


main()
