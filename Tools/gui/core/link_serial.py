# -*- coding: utf-8 -*-
"""core.link_serial — 基于 pyserial 的串口传输实现(USB-CDC 虚拟串口)。"""

from __future__ import annotations

import threading
from typing import List, Tuple

import serial
from serial.tools import list_ports

from .link import Link


def available_ports() -> List[Tuple[str, str]]:
    """返回 [(device, description), ...], 如 [('COM7', 'STM32 Virtual COM Port')]。"""
    out: List[Tuple[str, str]] = []
    for p in list_ports.comports():
        out.append((p.device, p.description or ""))
    return out


class SerialLink(Link):
    """线程读取串口, 按 \\n 切行后经 on_line 回调抛给输出面板。"""

    def __init__(self) -> None:
        super().__init__()
        self._ser = None
        self._th = None
        self._run = False
        self._buf = bytearray()

    @property
    def is_open(self) -> bool:
        return self._ser is not None and getattr(self._ser, "is_open", False)

    def open(self, port: str, baudrate: int = 115200, **kwargs) -> bool:
        self.close()
        try:
            self._ser = serial.Serial(port, int(baudrate), timeout=0.05)
        except Exception as exc:  # noqa: BLE001
            self._emit_status(False, str(exc))
            self._ser = None
            return False
        self._buf = bytearray()
        self._run = True
        self._th = threading.Thread(target=self._reader, daemon=True)
        self._th.start()
        self._emit_status(True, f"{port} @ {int(baudrate)}")
        return True

    def _reader(self) -> None:
        while self._run and self._ser is not None:
            try:
                data = self._ser.read(256)
            except Exception as exc:  # noqa: BLE001
                self._emit_status(False, str(exc))
                break
            if not data:
                continue
            self._buf.extend(data)
            while b"\n" in self._buf:
                line, _, rest = self._buf.partition(b"\n")
                self._buf = bytearray(rest)
                self._emit_line(line.decode("utf-8", errors="replace").rstrip("\r"))

    def send(self, cmd: str) -> None:
        if not self.is_open:
            return
        try:
            self._ser.write((cmd.rstrip("\r\n") + "\r\n").encode("ascii", "ignore"))
        except Exception as exc:  # noqa: BLE001
            self._emit_status(False, str(exc))

    def close(self) -> None:
        self._run = False
        th, self._th = self._th, None
        if th is not None:
            th.join(0.5)
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:  # noqa: BLE001
                pass
            self._ser = None
        self._emit_status(False, "")
