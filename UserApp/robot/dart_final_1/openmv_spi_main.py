# -*- coding: utf-8 -*-
"""
openmv_spi_main.py — OpenMV Cam H7 端: SPI 主机(controller), 向 STM32 SPI2 从机发 7 字节帧。
================================================================================
OpenMV 的 machine.SPI 只有【主机】模式, 所以由 OpenMV 提供 SCK/CS, STM32 做从机。

接线 (OpenMV H7 ↔ STM32F407 C 板):
    OpenMV P2 (SCK) -> STM32 PB13 (SPI2_SCK)
    OpenMV P0 (MOSI)-> STM32 PB15 (SPI2_MOSI)
    OpenMV P3 (CS)  -> STM32 PB12 (SPI2_NSS)
    GND             -> GND
    (OpenMV P1/MISO 不用)

帧格式(7 字节, 大端): AA 55 X_hi X_lo Y_hi Y_lo CRC8(SHT75 poly=0x31)
未识别: X=Y=0。

部署: 与 camara_ai.py 一起拷到 OpenMV 根目录, 命名为 main.py (或手动 import 运行)。
"""
import time
from machine import SPI, Pin

SPI_ID = 2
BAUD = 1_000_000          # STM32 从机可到几 MHz; 先保守用 1MHz
CS = Pin("P3", Pin.OUT, value=1)
spi = SPI(SPI_ID, baudrate=BAUD, polarity=0, phase=0, firstbit=SPI.MSB,
          sck=Pin("P2"), mosi=Pin("P0"), miso=Pin("P1"))

DEMO = True               # True=不用相机, x 自动来回扫; False=用 camara_ai 识别绿光


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
    CS.low()
    spi.write(pack(x, y, found))
    CS.high()


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
        x = 80 + (dt // 10) % 220          # 80..299 循环
        y = 120
        found = True
    send(x, y, found)
    time.sleep_ms(20)                      # ~50Hz
