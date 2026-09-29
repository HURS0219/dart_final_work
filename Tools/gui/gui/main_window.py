# -*- coding: utf-8 -*-
"""gui.main_window — 组装界面: 顶部传输/连接栏 + 左输入面板 + 右输出面板。

传输可选:
  - RTT (J-Link SWD)  默认; 只用 SWD 就能用(core.link_rtt)
  - Serial (COM)      板载 USB-CDC; core.link_serial
输入/输出面板与传输解耦, 只经 core.link 边界交互。
"""

from __future__ import annotations

from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (
    QComboBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

from core.link_serial import SerialLink, available_ports
from core.link_rtt import RttLink, pylink_available
from input.panel import InputPanel
from output.panel import OutputPanel

RTT_DEVICE_DEFAULT = "STM32F407IG"
RTT_SPEED_DEFAULT = "4000"


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Dart Test Console — dart_final_test_app")
        self.resize(1160, 740)

        self.link = None  # 当前 Link(RTT 或 Serial)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)

        # ---------- 顶部: 传输 + 连接 ----------
        bar = QHBoxLayout()
        bar.addWidget(QLabel("传输:"))
        self.cb_transport = QComboBox()
        self.cb_transport.addItems(["RTT (J-Link SWD)", "Serial (COM)"])
        bar.addWidget(self.cb_transport)

        # RTT 参数
        self.w_rtt = QWidget()
        rl = QHBoxLayout(self.w_rtt)
        rl.setContentsMargins(0, 0, 0, 0)
        rl.addWidget(QLabel("Device:"))
        self.ed_device = QLineEdit(RTT_DEVICE_DEFAULT)
        self.ed_device.setFixedWidth(140)
        rl.addWidget(self.ed_device)
        rl.addWidget(QLabel("Speed(kHz):"))
        self.ed_speed = QLineEdit(RTT_SPEED_DEFAULT)
        self.ed_speed.setFixedWidth(70)
        rl.addWidget(self.ed_speed)
        bar.addWidget(self.w_rtt)

        # Serial 参数
        self.w_serial = QWidget()
        sl = QHBoxLayout(self.w_serial)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.addWidget(QLabel("串口:"))
        self.cb_port = QComboBox()
        self.cb_port.setMinimumWidth(320)
        sl.addWidget(self.cb_port)
        self.btn_refresh = QPushButton("刷新")
        sl.addWidget(self.btn_refresh)
        sl.addWidget(QLabel("波特率:"))
        self.cb_baud = QComboBox()
        self.cb_baud.addItems(["115200", "230400", "460800", "921600", "9600"])
        sl.addWidget(self.cb_baud)
        bar.addWidget(self.w_serial)

        self.btn_conn = QPushButton("连接")
        bar.addWidget(self.btn_conn)
        bar.addStretch(1)
        self.lbl_status = QLabel("未连接")
        bar.addWidget(self.lbl_status)
        root.addLayout(bar)

        # ---------- 主体 ----------
        split = QSplitter(Qt.Horizontal)
        self.input_panel = InputPanel()
        self.output_panel = OutputPanel()
        split.addWidget(self.input_panel)
        split.addWidget(self.output_panel)
        split.setStretchFactor(0, 0)
        split.setStretchFactor(1, 1)
        split.setSizes([440, 720])
        root.addWidget(split, 1)

        # ---------- 事件 ----------
        self.cb_transport.currentIndexChanged.connect(self._on_transport_changed)
        self.btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_conn.clicked.connect(self.toggle_conn)
        self.input_panel.command.connect(self.on_command)
        self.output_panel.btn_stat.clicked.connect(lambda: self.on_command("STAT"))
        self.output_panel.btn_clear.clicked.connect(self.output_panel.text.clear)

        self._on_transport_changed()
        self.refresh_ports()

    # ---------- 传输切换 ----------
    def _on_transport_changed(self) -> None:
        rtt = self.cb_transport.currentIndex() == 0
        self.w_rtt.setVisible(rtt)
        self.w_serial.setVisible(not rtt)

    # ---------- 串口枚举 ----------
    def refresh_ports(self) -> None:
        cur = self.cb_port.currentData()
        self.cb_port.clear()
        for dev, desc in available_ports():
            self.cb_port.addItem("%s  %s" % (dev, desc), dev)
        idx = 0
        for i in range(self.cb_port.count()):
            t = self.cb_port.itemText(i).lower()
            if "stm32" in t or "virtual com" in t:
                idx = i
                break
        if cur is not None:
            for i in range(self.cb_port.count()):
                if self.cb_port.itemData(i) == cur:
                    idx = i
                    break
        if self.cb_port.count():
            self.cb_port.setCurrentIndex(idx)

    # ---------- 连接 ----------
    def _wire(self, link) -> None:
        link.on_line(self.output_panel.append_line)
        link.on_status(self.on_status)

    def toggle_conn(self) -> None:
        if self.link is not None and self.link.is_open:
            self.link.close()
            self.link = None
            return

        if self.cb_transport.currentIndex() == 0:  # RTT
            if not pylink_available():
                QMessageBox.warning(self, "Dart Test Console",
                                    "未安装 pylink-square。\n运行: pip install pylink-square")
                return
            dev = self.ed_device.text().strip() or RTT_DEVICE_DEFAULT
            try:
                speed = int(self.ed_speed.text().strip() or RTT_SPEED_DEFAULT)
            except ValueError:
                speed = int(RTT_SPEED_DEFAULT)
            link = RttLink()
            self._wire(link)
            self.link = link
            if not link.open(dev, speed):
                self.link = None
        else:  # Serial
            port = self.cb_port.currentData()
            if not port:
                QMessageBox.warning(self, "Dart Test Console",
                                    "没有可用串口。\n把板子 USB 插到电脑后点『刷新』。")
                return
            link = SerialLink()
            self._wire(link)
            self.link = link
            if not link.open(port, int(self.cb_baud.currentText())):
                self.link = None

    def on_command(self, cmd: str) -> None:
        if self.link is not None and self.link.is_open:
            self.link.send(cmd)
        else:
            self.output_panel.append_line("<<未连接, 未发送>> " + cmd)

    def on_status(self, connected: bool, message: str) -> None:
        self.lbl_status.setText(("已连接 " + message) if connected else ("未连接 " + (message or "")))
        self.btn_conn.setText("断开" if connected else "连接")
