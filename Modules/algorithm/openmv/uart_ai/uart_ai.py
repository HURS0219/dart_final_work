# -*- coding: utf-8 -*-
"""
uart_ai.py —— OpenMV 绿光坐标串口发送模块
================================================================================
作用：
    把 camara_ai 识别出的绿光坐标 (x, y) 打包成定长帧，通过 UART 发给 STM32。
    STM32 端用 Bsp/usart 的 DMA+空闲中断接收：
        注册串口实例时 recv_buff_size = UART_FRAME_LEN(=8)，
        在 module_callback 里校验帧头 + CRC8 后取出 x、y。

帧格式(定长 8 字节，大端)：
    ------------------------------------------------------------------
    | 0    | 1    | 2     | 3     | 4     | 5     | 6     | 7        |
    | 0xAA | 0x55 | X_hi  | X_lo  | Y_hi  | Y_lo  | flags | CRC8     |
    ------------------------------------------------------------------
    X = (X_hi<<8) | X_lo     绿光中心 x 像素 (0~319 @QVGA)
    Y = (Y_hi<<8) | Y_lo     绿光中心 y 像素 (0~239 @QVGA)
    flags  bit0: 1=本帧检测到目标, 0=目标丢失(x,y 无意义)
    CRC8   = crc8(byte0..byte6)  使用与 STM32 库一致的 SHT75 CRC8
             (poly=0x31, init=0x00, 不反转), STM32 可用 crc_8() 直接校验

示例(OpenMV 端)：
    from uart_ai import UartLink
    link = UartLink(uart_id=3, baudrate=115200)
    link.send(x, y, found)

注意：
    * OpenMV 不同型号 UART 引脚不同(如 H7 的 UART3 通常为 P4/P5)，请按板子接线，
      并保证 TX->STM32 RX、RX->STM32 TX、共地。
    * 波特率需与 STM32 端一致(默认 115200)。
================================================================================
"""

from pyb import UART


# ============================================================================
#                              协议常量
# ============================================================================

FRAME_HEADER_1 = 0xAA     # 帧头第 1 字节
FRAME_HEADER_2 = 0x55     # 帧头第 2 字节
UART_FRAME_LEN = 8        # 一帧总长度(STM32 recv_buff_size 必须等于它)

FLAG_TARGET_FOUND = 0x01  # flags: bit0 表示本帧是否检测到绿光


# ============================================================================
#                              CRC8 (SHT75)
# ============================================================================
def crc8(data):
    """计算 SHT75 标准 CRC8：多项式 0x31，初值 0x00，输入输出不反转。

    与 STM32 侧 Modules/algorithm/crc8 的 crc_8() 结果完全一致，
    因此 STM32 可直接用 crc_8(buf, 7) 校验本帧。
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x31) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


# ============================================================================
#                              串口链路类
# ============================================================================
class UartLink(object):
    """封装“打包 + 发送”的串口链路对象。"""

    def __init__(self, uart_id=3, baudrate=115200):
        """构造并打开串口。

        参数:
            uart_id : OpenMV 的 UART 编号(如 3)，引脚随型号而定
            baudrate: 波特率，需与 STM32 一致
        """
        # 8 位数据位、无校验、1 位停止位；timeout_char 设为 0 表示非阻塞写
        self.uart = UART(uart_id, baudrate=baudrate, bits=8,
                         parity=None, stop=1, timeout_char=100)

    # ------------------------------------------------------------------ #
    # 打包
    # ------------------------------------------------------------------ #
    @staticmethod
    def pack(x, y, found):
        """把坐标打包成 8 字节定长帧(bytearray)。

        参数:
            x, y : 像素坐标(0~65535，实际为图像分辨率范围)
            found: bool/0/1，是否检测到目标
        返回:
            bytearray，长度 = UART_FRAME_LEN
        """
        # 1) 限制坐标到 16 位范围，防止负数/越界
        xi = int(x) & 0xFFFF
        yi = int(y) & 0xFFFF

        # 2) 组装 flags
        flags = FLAG_TARGET_FOUND if found else 0x00

        # 3) 按“大端”填充前 7 字节
        frame = bytearray(UART_FRAME_LEN)
        frame[0] = FRAME_HEADER_1
        frame[1] = FRAME_HEADER_2
        frame[2] = (xi >> 8) & 0xFF
        frame[3] = xi & 0xFF
        frame[4] = (yi >> 8) & 0xFF
        frame[5] = yi & 0xFF
        frame[6] = flags

        # 4) 第 8 字节为前 7 字节的 CRC8
        frame[7] = crc8(frame[0:7])
        return frame

    # ------------------------------------------------------------------ #
    # 发送
    # ------------------------------------------------------------------ #
    def send(self, x, y, found):
        """打包并发送一帧，返回实际写入的字节数。"""
        return self.uart.write(self.pack(x, y, found))

    def send_frame(self, frame):
        """直接发送已打包好的帧(便于复用/测试)。"""
        return self.uart.write(frame)

    # ------------------------------------------------------------------ #
    # 关闭
    # ------------------------------------------------------------------ #
    def deinit(self):
        """关闭串口(释放资源)。"""
        try:
            self.uart.deinit()
        except Exception:
            pass
