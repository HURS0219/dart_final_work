# rtt_uart.py — 用 J-Link RTT 模拟 UART, 让 proto.Bus 在 PC 上原样工作(只需 SWD, 无需 ESP)
#
# 与固件约定: RTT 通道 1 = 发射架协议
#   下行(PC->MCU 命令): rtt_write(1, b"...\n")
#   上行(MCU->PC 遥测): rtt_read(1, N)
import os
import threading
import time

try:
    import pylink
except Exception as _e:  # noqa: BLE001
    pylink = None
    _IMP_ERR = _e

JLINK_DIR = r"C:\Program Files\SEGGER\JLink_V966"
DEVICE = os.environ.get("DART_RTT_DEVICE", "STM32F407IG")
SPEED = 4000
RTT_CH = 1


class UART:
    """兼容 machine.UART(uart_id, baudrate, tx, rx, rxbuf) 的构造签名(其余忽略)。"""

    def __init__(self, uart_id=1, baudrate=115200, tx=None, rx=None, rxbuf=1024, *a, **k):
        if pylink is None:
            raise RuntimeError("pylink-square 未安装: pip install pylink-square (%s)" % _IMP_ERR)
        self.ch = RTT_CH
        self.rxbuf = bytearray()
        self._lock = threading.Lock()
        self._run = True
        self._jl = self._make_jlink()
        self._jl.open()
        self._jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
        try:
            self._jl.set_speed(int(SPEED))
        except Exception:  # noqa: BLE001
            pass
        self._jl.connect(str(DEVICE))
        if self._jl.halted():
            self._jl.restart()
        self._jl.rtt_start()
        ok = False
        for _ in range(100):
            try:
                self._jl.rtt_start()
            except Exception:  # noqa: BLE001
                pass
            try:
                if self._jl.rtt_get_num_up_buffers() > 0:
                    ok = True
                    break
            except Exception:  # noqa: BLE001
                pass
            time.sleep(0.1)
        if not ok:
            raise RuntimeError("RTT 控制块未找到(固件没跑起来?)")
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()

    def _make_jlink(self):
        try:
            return pylink.JLink(log=False)
        except Exception:  # noqa: BLE001
            lib = pylink.library.Library(JLINK_DIR)
            return pylink.JLink(lib=lib, log=False)

    def _reader(self):
        while self._run:
            try:
                d = self._jl.rtt_read(self.ch, 1024)
            except Exception:  # noqa: BLE001
                break
            if d:
                if not isinstance(d, (bytes, bytearray)):
                    d = bytes(d)
                with self._lock:
                    self.rxbuf.extend(d)
            else:
                time.sleep(0.01)

    def read(self, n=None):
        with self._lock:
            if not self.rxbuf:
                return None
            data = bytes(self.rxbuf)
            self.rxbuf = bytearray()
            return data

    def any(self):
        with self._lock:
            return len(self.rxbuf)

    def write(self, s):
        if isinstance(s, str):
            s = s.encode("ascii", "ignore")
        try:
            self._jl.rtt_write(self.ch, s)
        except Exception:  # noqa: BLE001
            pass

    def deinit(self):
        self._run = False
        try:
            self._jl.rtt_stop()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._jl.close()
        except Exception:  # noqa: BLE001
            pass
