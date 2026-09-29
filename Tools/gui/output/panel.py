# -*- coding: utf-8 -*-
"""
output.panel — 输出面板: 把设备发来的每行文本打印出来, 并提供 STAT 按钮。

与输入面板完全解耦: 它只消费 Link.on_line 抛来的行, 以及发一条 "STAT" 命令。
"""

from __future__ import annotations

from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import (
    QHBoxLayout,
    QLabel,
    QPlainTextEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


class OutputPanel(QWidget):
    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)

        row = QHBoxLayout()
        self.btn_stat = QPushButton("STAT — 打印一次")
        self.btn_clear = QPushButton("清空")
        row.addWidget(self.btn_stat)
        row.addWidget(self.btn_clear)
        row.addWidget(QLabel("   (收到的串口文本)"))
        row.addStretch(1)
        root.addLayout(row)

        self.text = QPlainTextEdit()
        self.text.setReadOnly(True)
        self.text.setLineWrapMode(QPlainTextEdit.NoWrap)
        font = QFont("Consolas")
        font.setStyleHint(QFont.Monospace)
        font.setPointSize(9)
        self.text.setFont(font)
        self.text.setMaximumBlockCount(20000)
        root.addWidget(self.text, 1)

    def append_line(self, line: str) -> None:
        self.text.appendPlainText(line)
