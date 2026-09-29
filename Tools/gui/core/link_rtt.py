# -*- coding: utf-8 -*-
"""core.link_rtt — 基于 pylink-square 的 J-Link RTT 传输实现(仅需 SWD)。

- 命令: 写 RTT down channel 0  -> jl.rtt_write(0, cmd+"\\r\\n")
- 输出: 持续轮询 RTT up channel 0 -> jl.rtt_read(0, N), 按 \\n 切行

只需 J-Link 的 SWD 连接(无需板载 USB / UART)。
"""

from __future__ import annotations

import threading
import time
from typing import Optional

from .link import Link

try:
    import pylink
    _OK = True
except Exception:  # noqa: BLE001
    _OK = False

# J-Link 安装目录(找不到 DLL 时回退)
JLINK_DIR = r"C:\Program Files\SEGGER\JLink_V966"


def pylink_available() -> bool:
    return _OK


def _make_jlink():
    if not _OK:
        raise RuntimeError("pylink-square 未安装 (pip install pylink-square)")
    try:
        return pylink.JLink(log=False)
    except Exception:  # noqa: BLE001 - 显式指定 J-Link 目录
        lib = pylink.library.Library(JLINK_DIR)
        return pylink.JLink(lib=lib, log=False)


class RttLink(Link):
    def __init__(self) -> None:
        super().__init__()
        self._jl = None
        self._th: Optional[threading.Thread] = None
        self._run = False
        self._buf = bytearray()
        self._channel = 0

    @property
    def is_open(self) -> bool:
        return self._jl is not None

    def open(self, device: str = "STM32F407IG", speed: int = 4000, channel: int = 0, **kwargs) -> bool:
        self.close()
        jl = None
        try:
            jl = _make_jlink()
            jl.open()
            jl.set_tif(pylink.enums.JLinkInterfaces.SWD)
            try:
                jl.set_speed(int(speed))
            except Exception:  # noqa: BLE001
                pass
            jl.connect(str(device))
            if jl.halted():
                jl.restart()
            jl.rtt_start()
            ok = False
            for _ in range(30):
                try:
                    if jl.rtt_get_num_up_buffers() > 0:
                        ok = True
                        break
                except Exception:  # noqa: BLE001
                    pass
                time.sleep(0.1)
            if not ok:
                raise RuntimeError("RTT 控制块未找到(固件没跑起来?)")
            self._jl = jl
            self._channel = int(channel)
            self._buf = bytearray()
            self._run = True
            self._th = threading.Thread(target=self._reader, daemon=True)
            self._th.start()
            self._emit_status(True, "RTT %s @ %dkHz" % (device, int(speed)))
            return True
        except Exception as exc:  # noqa: BLE001
            try:
                if jl is not None:
                    jl.close()
            except Exception:  # noqa: BLE001
                pass
            self._jl = None
            self._emit_status(False, str(exc))
            return False

    def _reader(self) -> None:
        ch = self._channel
        while self._run and self._jl is not None:
            try:
                data = self._jl.rtt_read(ch, 1024)
            except Exception as exc:  # noqa: BLE001
                self._emit_status(False, str(exc))
                break
            if data:
                if not isinstance(data, (bytes, bytearray)):
                    data = bytes(data)
                self._buf.extend(data)
                while b"\n" in self._buf:
                    line, _, rest = self._buf.partition(b"\n")
                    self._buf = bytearray(rest)
                    self._emit_line(line.decode("utf-8", errors="replace").rstrip("\r"))
            else:
                time.sleep(0.01)

    def send(self, cmd: str) -> None:
        if self._jl is None:
            return
        try:
            self._jl.rtt_write(self._channel, (cmd.rstrip("\r\n") + "\r\n").encode("ascii", "ignore"))
        except Exception as exc:  # noqa: BLE001
            self._emit_status(False, str(exc))

    def close(self) -> None:
        self._run = False
        th, self._th = self._th, None
        if th is not None:
            th.join(0.5)
        jl, self._jl = self._jl, None
        if jl is not None:
            try:
                jl.rtt_stop()
            except Exception:  # noqa: BLE001
                pass
            try:
                jl.close()
            except Exception:  # noqa: BLE001
                pass
        self._emit_status(False, "")
