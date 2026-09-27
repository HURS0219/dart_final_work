# -*- coding: utf-8 -*-
"""
logger_ai.py —— OpenMV 复盘记录模块（CSV 飞行数据 + AVI 录像 + 时间戳命名）
================================================================================
作用（借鉴深大 main_final.py 的 receive_flight_data_csv / rename_record_file）：
    把 OpenMV 当成“黑匣子”：一次发射同时留下
      * AVI 视频：摄像头视角的弹道/目标画面；
      * data.csv：STM32 通过同一 UART 回传的飞行数据(ASCII 文本行)。
    两者用 STM32 下发的时间戳统一命名，便于赛后对齐复盘。

对外接口：
    rec = VideoRecorder("record")      # 录像
    rec.start()                        # 开始
    rec.write(img)                     # 逐帧写(可每 N 帧写一次省卡)
    rec.close()                        # 结束
    rec.rename_with_timestamp(link)    # 读时间戳改名 record_<ts>.avi

    log = FlightLogger("data.csv")     # 飞行数据
    log.receive_csv(link, idle_timeout_ms=1000)   # 收 STM32 文本 -> CSV

注意：本文件依赖 OpenMV 的 mjpeg 模块与 SD 卡；主机端无法运行，仅语法检查。
================================================================================
"""

import os
import time

try:
    import mjpeg
    _HAS_MJPEG = True
except Exception:
    _HAS_MJPEG = False


# ============================================================================
#                              录像
# ============================================================================
class VideoRecorder(object):
    """AVI 录像器：包装 mjpeg.Mjpeg。"""

    def __init__(self, base_name="record", ext=".avi"):
        self.base_name = base_name
        self.ext = ext
        self._m = None
        self.filename = None

    def _next_filename(self):
        """找一个可用的递增文件名 recordNN.avi。"""
        max_num = 0
        for f in os.listdir():
            if f.startswith(self.base_name) and f.endswith(self.ext):
                try:
                    n = int(f[len(self.base_name):-len(self.ext)])
                    if n > max_num:
                        max_num = n
                except Exception:
                    continue
        return "%s%02d%s" % (self.base_name, max_num + 1, self.ext)

    def start(self, filename=None):
        """开始录像，返回文件名。"""
        if not _HAS_MJPEG:
            self.filename = None
            return None
        self.filename = filename or self._next_filename()
        self._m = mjpeg.Mjpeg(self.filename)
        return self.filename

    def write(self, img):
        """写入一帧(调用方自行控制频率，如每 5 帧写一次)。"""
        if self._m is not None:
            self._m.write(img)

    def close(self):
        """结束录像。"""
        if self._m is not None:
            self._m.close()
            self._m = None

    def rename_with_timestamp(self, link, timeout_s=10):
        """等 STM32 发来 15 字节时间戳，把录像改成 record_<时间戳>.avi。

        参数 link: UartLink 实例(需有 uart.read / uart.any)。返回最终文件名。
        """
        if self.filename is None:
            return None
        t0 = time.ticks_ms()
        while not link.uart.any():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_s * 1000:
                return self.filename
            time.sleep_ms(5)
        ts = link.uart.read(15)
        try:
            ts = ts.decode("ASCII")
        except Exception:
            ts = ""
        # 深大约定：时间戳首字符为 '2' 才是合法时间(如 2026...)
        if ts and ts[0] == "2":
            new_name = "record_%s.avi" % ts
            try:
                os.rename(self.filename, new_name)
                self.filename = new_name
            except Exception:
                pass
        return self.filename


# ============================================================================
#                              飞行数据 CSV
# ============================================================================
class FlightLogger(object):
    """接收 STM32 通过 UART 发来的 ASCII 文本行，存成 CSV。"""

    def __init__(self, filename="data.csv", chunk=64):
        self.filename = filename
        self.chunk = chunk

    def receive_csv(self, link, idle_timeout_ms=1000):
        """持续接收直到串口空闲超过 idle_timeout_ms，之后停止。返回文件名。"""
        buf = ""
        started = False
        last_rx = time.ticks_ms()

        with open(self.filename, "w") as f:
            while True:
                if link.uart.any():
                    started = True
                    last_rx = time.ticks_ms()
                    raw = link.uart.read(self.chunk)
                    if raw:
                        buf += raw.decode("ASCII", "ignore")
                        while "\n" in buf:
                            line, buf = buf.split("\n", 1)
                            if line.strip():
                                f.write(line + "\n")

                if started and time.ticks_diff(time.ticks_ms(), last_rx) > idle_timeout_ms:
                    break
                time.sleep_ms(5)

            if buf.strip():
                f.write(buf + "\n")

        return self.filename
