# -*- coding: utf-8 -*-
"""gui.main_window — 组装界面: 顶部串口连接栏 + 左输入面板 + 右输出面板。"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from core.link_serial import SerialLink, available_ports
from input.panel import InputPanel
from output.panel import OutputPanel


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Dart Test Console — dart_final_test_app")
        self.resize(1120, 720)

        self.link = SerialLink()

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # ---- 顶部: 连接栏 ----
        bar = QHBoxLayout()
        bar.addWidget(QLabel("串口:"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(340)
        bar.addWidget(self.cb_port)
        self.btn_refresh = QPushButton("刷新")
        bar.addWidget(self.btn_refresh)
        bar.addWidget(QLabel("波特率:"))
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["115200", "230400", "460800", "921600", "9600"])
        bar.addWidget(self.cb_baud)
        self.btn_conn = QPushButton("连接")
        bar.addWidget(self.btn_conn)
        bar.addStretch(1)
        self.lbl_status = QLabel("未连接")
        bar.addWidget(self.lbl_status)
        root.addLayout(bar)

        # ---- 主体: 左输入 / 右输出 ----
        split = QSplitter(Qt.Horizontal)
        self.input_panel = InputPanel()
        self.output_panel = OutputPanel()
        split.addWidget(self.input_panel)
        split.addWidget(self.output_panel)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([430, 690])
        root.addWidget(split, 1)

        # ---- 连线 ----
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn.clicked.connect(self.toggle_conn)
        self.input_panel.command.connect(self.on_command)
        self.output_panel.btn_stat.clicked.connect(lambda: self.on_command("STAT"))
        self.output_panel.btn_clear.clicked.connect(self.output_panel.text.clear)
        self.link.on_line(self.output_panel.append_line)
        self.link.on_status(self.on_status)

        self.refresh_ports()

    # ---- 串口 ----
    def refresh_ports(self) -> None:
        cur = self.cb_port.currentData()
        self.cb_port.clear()
        for dev, desc in available_ports():
            self.cb_port.addItem("%s  %s" % (dev, desc), dev)
        idx = 0
        for i in range(self.cb_port.count()):
            t = self.cb_port.itemText(i).lower()
            if "stm32" in t or "virtual com" in t:  # 优先选板载 CDC
                idx = i
                break
        if cur is not None:
            for i in range(self.cb_port.count()):
                if self.cb_port.itemData(i) == cur:
                    idx = i
                    break
        if self.cb_port.count():
            self.cb_port.setCurrentIndex(idx)

    def toggle_conn(self) -> None:
        if self.link.is_open:
            self.link.close()
            return
        dev = self.cb_port.currentData()
        if not dev:
            QMessageBox.warning(self, "Dart Test Console",
                                "没有可用串口。\n请把板子 USB 插到电脑, 然后点『刷新』。")
            return
        self.link.open(dev, int(self.cb_baud.currentText()))

    def on_command(self, cmd: str) -> None:
        if self.link.is_open:
            self.link.send(cmd)
        else:
            self.output_panel.append_line("<<未连接, 未发送>> " + cmd)

    def on_status(self, connected: bool, message: str) -> None:
        self.lbl_status.setText(("已连接 " + message) if connected else ("未连接 " + (message or "")))
        self.btn_conn.setText("断开" if connected else "连接")
