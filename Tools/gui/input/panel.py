# -*- coding: utf-8 -*-
"""
input.panel — 输入面板(只负责"产生命令", 经 command 信号交给 Link 发出)。

三组, 全部复用 input.slider.SliderControl:
  1) target: 滑条控制 x(0..319) -> TGT,<x>,120 ; 配绿光点水平预览; + TGTN(丢失)
  2) 舵面:   4 根滑条(-35..35 逻辑角) -> FIN,<ch>,<deg>
  3) 混控:   p/y/r 三根滑条(-1..1)  -> FMIX,<p>,<y>,<r>

命令格式与 dart_final_test_app 的串口/RTT 控制台一致(见该 app 的 README)。
"""

from __future__ import annotations

from PyQt5.QtCore import Qt, pyqtSignal
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import (
    QGroupBox,
    QHBoxLayout,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from .slider import SliderControl

# 目标中心像素范围(QVGA 320x240); y 固定 120(只调 x, 不涉及 pitch)
TARGET_X_MAX = 319
TARGET_Y_FIX = 120
# 舵面逻辑角限位(与 servo_mix_ai 的 SERVO_MIX_MAX_DEG 一致)
FIN_LIMIT_DEG = 35.0


class TargetPreview(QWidget):
    """绿光点横坐标预览: 一个水平"画面"条, 小圆点随 x 左右移动。"""

    def __init__(self, x_max: int = TARGET_X_MAX, parent=None) -> None:
        super().__init__(parent)
        self._x_max = max(1, int(x_max))
        self._x = self._x_max / 2.0
        self.setMinimumHeight(64)

    def set_x(self, x: float) -> None:
        self._x = x
        self.update()

    def paintEvent(self, _event) -> None:  # noqa: N802
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        p.fillRect(0, 0, w, h, QColor(18, 18, 18))
        y = int(h * 0.68)
        margin = 12
        # 画面水平基线
        p.setPen(QPen(QColor(80, 80, 80), 1))
        p.drawLine(margin, y, w - margin, y)
        # 中点标记
        cx = margin + (w - 2 * margin) * 0.5
        p.setPen(QPen(QColor(70, 70, 70), 1, Qt.DashLine))
        p.drawLine(int(cx), 6, int(cx), h - 6)
        # 绿光点
        gx = margin + (w - 2 * margin) * (max(0.0, min(self._x_max, self._x)) / self._x_max)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(0, 220, 0))
        p.drawEllipse(int(gx) - 6, y - 6, 12, 12)
        p.end()


class InputPanel(QWidget):
    """输入面板: 产生命令串(不关心传输)。"""

    command = pyqtSignal(str)

    def __init__(self, parent=None) -> None:
        super().__init__(parent)
        root = QVBoxLayout(self)

        # 1) target
        g1 = QGroupBox("target — 绿光点 x 注入   TGT,<x>,120")
        l1 = QVBoxLayout(g1)
        self.sl_target = SliderControl(
            "x", 0, TARGET_X_MAX, TARGET_X_MAX // 2, 1,
            fmt=lambda v: "TGT,%d,%d" % (int(round(v)), TARGET_Y_FIX), unit="px",
        )
        l1.addWidget(self.sl_target)
        self.preview = TargetPreview()
        l1.addWidget(self.preview)
        row1 = QHBoxLayout()
        self.btn_lost = QPushButton("目标丢失 (TGTN)")
        row1.addWidget(self.btn_lost)
        row1.addStretch(1)
        l1.addLayout(row1)
        root.addWidget(g1)

        # 2) 舵面逻辑角
        g2 = QGroupBox("舵面逻辑角   FIN,<ch>,<deg>")
        l2 = QVBoxLayout(g2)
        self.sl_fins = []
        for ch in range(4):
            s = SliderControl(
                "fin%d" % ch, -FIN_LIMIT_DEG, FIN_LIMIT_DEG, 0.0, 0.5,
                fmt=(lambda v, c=ch: "FIN,%d,%.2f" % (c, v)), unit="deg",
            )
            l2.addWidget(s)
            self.sl_fins.append(s)
        root.addWidget(g2)

        # 3) 混控 PYR
        g3 = QGroupBox("混控注入   FMIX,<p>,<y>,<r>")
        l3 = QVBoxLayout(g3)
        self.sl_p = SliderControl("pitch", -1.0, 1.0, 0.0, 0.01)
        self.sl_y = SliderControl("yaw", -1.0, 1.0, 0.0, 0.01)
        self.sl_r = SliderControl("roll", -1.0, 1.0, 0.0, 0.01)
        for s in (self.sl_p, self.sl_y, self.sl_r):
            l3.addWidget(s)
        root.addWidget(g3)

        root.addStretch(1)

        # ---- 连线 ----
        self.sl_target.valueChanged.connect(self.preview.set_x)
        self.sl_target.commandReady.connect(self.command)
        self.btn_lost.clicked.connect(lambda: self.command.emit("TGTN"))
        for s in self.sl_fins:
            s.commandReady.connect(self.command)
        for s in (self.sl_p, self.sl_y, self.sl_r):
            s.set_formatter(self._mix_command)  # 三者其一变化 -> 发完整 FMIX
            s.commandReady.connect(self.command)

    def _mix_command(self, _v: float) -> str:
        return "FMIX,%.3f,%.3f,%.3f" % (self.sl_p.value(), self.sl_y.value(), self.sl_r.value())
