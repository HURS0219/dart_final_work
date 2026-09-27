# -*- coding: utf-8 -*-
"""
uart_ai.py —— OpenMV 绿光坐标串口模块（9 字节 CRC8 帧 + 握手/触发）
================================================================================
作用：
    1) 把坐标 (x,y,w,h) 打包成 9 字节定长帧发给 STM32；
    2) 实现与 STM32 的握手/触发协议，让 STM32 决定视觉何时开始、用哪个轴，
       并配合 main.py 里的 sensor.sleep() 发射前休眠省电降温。

帧格式(定长 9 字节，大端)：
    [0]0xAA [1]0x55 [2]X_hi [3]X_lo [4]Y_hi [5]Y_lo [6]W [7]H [8]CRC8
    CRC8: SHT75(poly=0x31, init=0)，与 STM32 crc8 库一致
    丢失: W=H=0, X=Y=0

握手协议(STM32 -> OpenMV, 单字节命令)：
    0x11 = 选 yaw 轴      0x22 = 选 pitch 轴
    0xAB = 唤醒(结束 sensor.sleep)
    0x55 = 普通模式       0xFF = 比赛模式
OpenMV -> STM32:
    0xFF 'R''P''D''A''R''T'   就绪握手(启动完成)

示例：
    link = UartLink()
    axis = link.wait_axis()      # 阻塞等 0x11/0x22
    link.send_ready()            # 回就绪
    link.wait_wake()             # 等 0xAB 唤醒
    mode = link.wait_mode()      # 等 0x55/0xFF
    link.send(x, y, w, h, found)
================================================================================
"""

from pyb import UART


# ============================================================================
#                              协议常量
# ============================================================================
FRAME_HEADER_1 = 0xAA
FRAME_HEADER_2 = 0x55
UART_FRAME_LEN = 9

CMD_AXIS_YAW = 0x11
CMD_AXIS_PITCH = 0x22
CMD_WAKE = 0xAB
CMD_MODE_NORMAL = 0x55
CMD_MODE_COMPETITION = 0xFF

READY_BANNER = bytes([0xFF, ord('R'), ord('P'), ord('D'), ord('A'), ord('R'), ord('T')])


def crc8(data):
    """SHT75 CRC8：多项式 0x31，初值 0x00，输入输出不反转。"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


class UartLink(object):
    """串口链路：打包发送 + 握手/触发。"""

    def __init__(self, uart_id=3, baudrate=115200):
        self.uart = UART(uart_id, baudrate=baudrate, bits=8,
                         parity=None, stop=1, timeout_char=100)

    # --------------------------- 打包 / 发送 --------------------------- #
    @staticmethod
    def pack(x, y, w, h, found):
        """打包 9 字节帧(bytearray)。"""
        xi, yi = int(x) & 0xFFFF, int(y) & 0xFFFF
        wi, hi = int(w) & 0xFF, int(h) & 0xFF
        if not found:
            xi = yi = wi = hi = 0

        frame = bytearray(UART_FRAME_LEN)
        frame[0] = FRAME_HEADER_1
        frame[1] = FRAME_HEADER_2
        frame[2] = (xi >> 8) & 0xFF
        frame[3] = xi & 0xFF
        frame[4] = (yi >> 8) & 0xFF
        frame[5] = yi & 0xFF
        frame[6] = wi
        frame[7] = hi
        frame[8] = crc8(frame[0:8])
        return frame

    def send(self, x, y, w, h, found):
        """打包并发送一帧。"""
        return self.uart.write(self.pack(x, y, w, h, found))

    def send_ready(self):
        """发送就绪握手串。"""
        return self.uart.write(READY_BANNER)

    # --------------------------- 接收(握手) ---------------------------- #
    def read_byte(self):
        """非阻塞读 1 字节，无数据返回 None。"""
        if self.uart.any():
            b = self.uart.read(1)
            if b:
                return ord(b)
        return None

    def wait_byte(self, codes, timeout_ms=None):
        """等待 codes 中的某个字节；超时返回 None。"""
        t0 = 0
        while True:
            b = self.read_byte()
            if b is not None and b in codes:
                return b
            if timeout_ms is not None and b is None:
                # 简易超时(依赖调用方传入合理的 timeout_ms)
                t0 += 1
                if t0 > timeout_ms:
                    return None

    def wait_axis(self):
        """等待 STM32 指定制导轴，返回 'yaw' 或 'pitch'。"""
        while True:
            b = self.read_byte()
            if b == CMD_AXIS_YAW:
                return 'yaw'
            if b == CMD_AXIS_PITCH:
                return 'pitch'

    def wait_wake(self):
        """等待 0xAB 唤醒命令。"""
        while True:
            if self.read_byte() == CMD_WAKE:
                return True

    def wait_mode(self):
        """等待模式选择，返回 'normal' 或 'competition'。"""
        while True:
            b = self.read_byte()
            if b == CMD_MODE_NORMAL:
                return 'normal'
            if b == CMD_MODE_COMPETITION:
                return 'competition'

    def deinit(self):
        try:
            self.uart.deinit()
        except Exception:
            pass
