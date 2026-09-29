# -*- coding: utf-8 -*-
"""
openmv_uart_main.py — OpenMV Cam H7 端: 3 线串口发送 7 字节帧给 STM32 (USART3)。
================================================================================
匹配 STM32 侧 USART3 配置: 100000 baud, 8 数据位 + 偶校验(Even), 1 停止位。
(即 STM32 的 WordLength=9B + Parity=EVEN == OpenMV bits=8 + parity=0)

接线 (3 线):
    OpenMV P4 (UART3_TX) -> STM32 PC11 (USART3_RX)
    OpenMV P5 (UART3_RX) -> STM32 PC10 (USART3_TX)   # 发送方向可不接, 但建议接
    GND                  <-> GND

帧格式(7 字节, 大端): AA 55 X_hi X_lo Y_hi Y_lo CRC8(SHT75 poly=0x31)
未识别: X=Y=0。

部署: 与 camara_ai.py 一起拷到 OpenMV 根目录, 命名为 main.py(或手动运行)。
"""
import time
from pyb import UART

UART_ID = 3
BAUD = 100000
uart = UART(UART_ID, baudrate=BAUD, bits=8, parity=0, stop=1, timeout_char=100)

DEMO = True          # True=不用相机, x 自动来回扫; False=用 camara_ai 识别绿光


def crc8(data):
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x31) & 0xFF if (crc & 0x80) else ((crc << 1) & 0xFF)
    return crc


def pack(x, y, found):
    if not found:
        x = y = 0
    xi, yi = int(x) & 0xFFFF, int(y) & 0xFFFF
    b = bytearray(7)
    b[0] = 0xAA
    b[1] = 0x55
    b[2] = (xi >> 8) & 0xFF
    b[3] = xi & 0xFF
    b[4] = (yi >> 8) & 0xFF
    b[5] = yi & 0xFF
    b[6] = crc8(b[0:6])
    return b


def send(x, y, found):
    uart.write(pack(x, y, found))


det = None
if not DEMO:
    from camara_ai import GreenLight
    det = GreenLight()
    det.init_sensor()

t0 = time.ticks_ms()
while True:
    if det is not None:
        x, y, w, h, found = det.detect()
    else:
        dt = time.ticks_diff(time.ticks_ms(), t0)
        x = 80 + (dt // 10) % 220
        y = 120
        found = True
    send(x, y, found)
    time.sleep_ms(20)
