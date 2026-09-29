# -*- coding: utf-8 -*-
"""
core.link — 输入/输出面板 与 传输层 之间的【隔离边界】。

设计要点:
  - 输入面板只依赖 Link.send(cmd)  —— 把命令发出去;
  - 输出面板只依赖 Link.on_line(cb) —— 收进来的每一行文本;
  - 二者互不 import, 可并行/隔离开发。
具体传输(RTT / 串口 / ...)由子类实现, 面板完全不关心。
"""

from __future__ import annotations

from typing import Callable


class Link:
    """传输层抽象基类(隔离边界)。"""

    def __init__(self) -> None:
        self._line_cbs: list = []
        self._status_cbs: list = []

    # ---- 面板调用 ----
    def on_line(self, cb: Callable[[str], None]) -> None:
        """注册"收到一整行文本"回调(输出面板用)。"""
        self._line_cbs.append(cb)

    def on_status(self, cb: Callable[[bool, str], None]) -> None:
        """注册连接状态回调: cb(connected, message)。"""
        self._status_cbs.append(cb)

    @property
    def is_open(self) -> bool:
        raise NotImplementedError

    def open(self, **kwargs) -> bool:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def send(self, cmd: str) -> None:
        """发送一条命令(不含换行; 由实现补 \\r\\n)。"""
        raise NotImplementedError

    # ---- 子类回调 ----
    def _emit_line(self, line: str) -> None:
        for cb in list(self._line_cbs):
            try:
                cb(line)
            except Exception:
                pass

    def _emit_status(self, connected: bool, message: str = "") -> None:
        for cb in list(self._status_cbs):
            try:
                cb(connected, message)
            except Exception:
                pass
