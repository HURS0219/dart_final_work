# -*- coding: utf-8 -*-
"""
main.py —— OpenMV 端整合（Hybrid 版：识别 + 握手 + 发送 + 复盘记录）
================================================================================
流程（借鉴深大 main_final.py）：
    1. 等 STM32 指定制导轴(0x11=yaw / 0x22=pitch)；
    2. 初始化摄像头，回就绪串 "RPDART"；
    3. sensor.sleep(True) 休眠省电降温 → 等 0xAB 唤醒 → 等 0x55/0xFF 选模式；
    4. 主循环：动态 ROI 识别 → 7 字节 CRC 帧发送(仅 x,y) → LED 指示 → (可选)录像；
    5. 结束后：接收 STM32 文本存 data.csv，并把录像按时间戳改名。

部署：把本文件与 camara_ai.py / uart_ai.py / logger_ai.py 拷到 OpenMV 根目录。
================================================================================
"""

import time

from camara_ai import GreenLight
from uart_ai import UartLink
from logger_ai import VideoRecorder, FlightLogger

try:
    from pyb import LED, sensor
    _led_run = LED("LED_BLUE")
except Exception:
    _led_run = None


# ============================================================================
#                              可调参数
# ============================================================================
UART_ID = 3
UART_BAUDRATE = 115200

SEND_HZ = 50                 # 发送频率
RECORD_ENABLE = False        # 是否录像(需 SD 卡)
RECORD_EVERY_N = 5           # 每 N 帧录一帧
RUN_TIME_MS = None           # 主循环时长；None=一直跑(比赛用)
CSV_ENABLE = True            # 结束后是否收飞行数据


def main():
    # 1) 串口 + 等待 STM32 指定轴
    link = UartLink(uart_id=UART_ID, baudrate=UART_BAUDRATE)
    axis = link.wait_axis()          # 必须消费该握手字节(STM32 指定本相机负责的制导轴)
    if axis is None:
        axis = "yaw"                 # 兜底: 未收到选择时默认 yaw
    print("[main] guidance axis:", axis)

    # 2) 初始化摄像头，回就绪
    detector = GreenLight()
    detector.init_sensor()
    link.send_ready()

    # 3) 休眠等待发射触发(0xAB)，然后选模式(0x55/0xFF)
    sensor.sleep(True)
    link.wait_wake()
    sensor.sleep(False)
    mode = link.wait_mode()
    if mode is None:
        mode = "normal"              # 兜底

    # 4) 录像初始化(可选)
    rec = None
    if RECORD_ENABLE:
        rec = VideoRecorder("record")
        rec.start()

    # 5) 主循环
    period_ms = int(1000 / SEND_HZ)
    start = time.ticks_ms()
    frame_cnt = 0
    while True:
        t_start = time.ticks_ms()
        frame_cnt += 1

        # 5.1 识别(动态 ROI)
        x, y, w, h, found = detector.detect()

        # 5.2 发送 7 字节 CRC 帧 (仅目标中心 x,y; 不再发框尺寸)
        link.send(x, y, found)

        # 5.3 蓝灯心跳(每 25 帧翻转)
        if _led_run is not None and frame_cnt % 25 == 0:
            _led_run.toggle()

        # 5.4 录像(每 N 帧写一帧)
        if rec is not None and frame_cnt % RECORD_EVERY_N == 0:
            rec.write(sensor.snapshot())

        # 5.5 运行时长控制
        if RUN_TIME_MS is not None and time.ticks_diff(time.ticks_ms(), start) > RUN_TIME_MS:
            break

        # 5.6 节流
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < period_ms:
            time.sleep_ms(period_ms - elapsed)

    # 6) 收尾：关录像 + 按时间戳改名 + 收飞行数据
    if rec is not None:
        rec.close()
        if mode == "normal":
            rec.rename_with_timestamp(link)
    if CSV_ENABLE and mode == "competition":
        FlightLogger("data.csv").receive_csv(link)


if __name__ == "__main__":
    main()
