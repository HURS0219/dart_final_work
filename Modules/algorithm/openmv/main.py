# -*- coding: utf-8 -*-
"""
main.py —— OpenMV 端整合示例（绿光识别 + 串口发送）
================================================================================
本文件是“怎么把 camara_ai 和 uart_ai 串起来用”的示例入口。

部署方式：
    把本文件与 camara_ai.py、uart_ai.py 一起拷到 OpenMV 的根目录(U盘/IDE 下载)，
    摄像头会从根目录 import 这两个模块。两个模块源码分别在
    Modules/algorithm/openmv/camara_ai/ 和 .../uart_ai/ 下。

数据流：
    摄像头 -> GreenLight.detect() -> (x, y, found) -> UartLink.send() -> STM32(Bsp/usart)

硬件接线：
    OpenMV TX  -> STM32 RX
    OpenMV RX  -> STM32 TX (本单向发送场景可不接)
    GND        -> GND
================================================================================
"""

import time

from camara_ai import GreenLight
from uart_ai import UartLink


# ============================================================================
#                              可调参数
# ============================================================================
SEND_HZ = 50           # 发送频率(Hz)，一般 30~100 足够视觉闭环
UART_ID = 3            # OpenMV 使用的 UART 编号(引脚随型号而定)
UART_BAUDRATE = 115200 # 必须与 STM32 端一致


def main():
    """主循环：识别 -> 发送，按固定频率运行。"""
    # 1) 初始化检测器与串口
    detector = GreenLight()
    detector.init_sensor()               # 摄像头初始化(含等待自动曝光稳定)

    link = UartLink(uart_id=UART_ID, baudrate=UART_BAUDRATE)

    # 2) 计算发送周期(ms)
    period_ms = int(1000 / SEND_HZ)

    # 3) 主循环
    while True:
        t_start = time.ticks_ms()

        # 3.1 取一帧并检测绿光
        x, y, found = detector.detect()

        # 3.2 打包并发送坐标
        link.send(x, y, found)

        # 3.3 简单节流，保持固定发送频率
        elapsed = time.ticks_diff(time.ticks_ms(), t_start)
        if elapsed < period_ms:
            time.sleep_ms(period_ms - elapsed)


if __name__ == "__main__":
    main()
