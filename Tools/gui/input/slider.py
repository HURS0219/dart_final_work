# -*- coding: utf-8 -*-
"""
input.slider — 通用【滑条接口对象】SliderControl

对象输入: 两端极值 lo / hi  +  当前的实际用户输入值 value
对象输出: 由 value 生成"串口命令串"(通过 formatter 注入, 例如 TGT/FIN/FMIX)
        - 拖动时【限频】发 commandReady(~20Hz), 松手时立即再发一次, 避免刷爆串口。

四组输入(target / 4×fin / mix PYR)全部复用本对象。
"""

from __future__ import annotations

from typing import Callable, Optional

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import QDoubleSpinBox, QGridLayout, QLabel, QSlider, QWidget


class SliderControl(QWidget):
    """滑条接口对象: 极值(lo,hi) + 实际输入值(value) -> 命令串。"""

    valueChanged = pyqtSignal(float)   # 立即(用于预览等)
    commandReady = pyqtSignal(str)     # 限频后的命令串

    def __init__(
        self,
        label: str,
        lo: float,
        hi: float,
        value: float = 0.0,
        step: float = 1.0,
        fmt: Optional[Callable[[float], str]] = None,
        unit: str = "",
        throttle_ms: int = 50,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self.label = label
        self.lo = float(lo)
        self.hi = float(hi)
        self._step = float(step) if step else 1.0
        self._fmt = fmt
        self._unit = unit
        self._ticks = max(1, int(round((self.hi - self.lo) / self._step)))

        self._timer = QTimer(self)
        self._timer.setSingleShot(True)
        self._timer.setInterval(max(0, int(throttle_ms)))
        self._timer.timeout.connect(self._emit_command)

        self._lab = QLabel(label)
        self._slider = QSlider(Qt.Horizontal)
        self._slider.setRange(0, self._ticks)
        self._spin = QDoubleSpinBox()
        self._spin.setDecimals(self._decimals())
        self._spin.setSingleStep(self._step)
        self._spin.setRange(self.lo, self.hi)

        lay = QGridLayout(self)
        lay.setContentsMargins(2, 2, 2, 2)
        lay.addWidget(self._lab, 0, 0)
        lay.addWidget(self._slider, 0, 1)
        lay.addWidget(self._spin, 0, 2)
        if unit:
            lay.addWidget(QLabel(unit), 0, 3)

        self._slider.valueChanged.connect(self._on_slide)
        self._slider.sliderReleased.connect(self._emit_command)
        self._spin.valueChanged.connect(self._on_spin)
        self.set_value(value)

    # ---- 基本信息 ----
    def _decimals(self) -> int:
        s, d = self._step, 0
        while d < 3 and abs(s - round(s)) > 1e-9:
            s *= 10.0
            d += 1
        return d

    def value(self) -> float:
        return self.lo + self._slider.value() * self._step

    def set_value(self, v: float) -> None:
        v = max(self.lo, min(self.hi, float(v)))
        tick = int(round((v - self.lo) / self._step))
        self._slider.blockSignals(True)
        self._spin.blockSignals(True)
        self._slider.setValue(tick)
        self._spin.setValue(v)
        self._slider.blockSignals(False)
        self._spin.blockSignals(False)

    def set_formatter(self, fmt: Callable[[float], str]) -> None:
        self._fmt = fmt

    def command(self) -> str:
        if self._fmt is not None:
            return self._fmt(self.value())
        return "%s,%g" % (self.label, self.value())

    # ---- 交互 ----
    def _on_slide(self, tick: int) -> None:
        v = self.lo + tick * self._step
        self._spin.blockSignals(True)
        self._spin.setValue(v)
        self._spin.blockSignals(False)
        self.valueChanged.emit(v)
        self._timer.start()

    def _on_spin(self, v: float) -> None:
        tick = int(round((v - self.lo) / self._step))
        self._slider.blockSignals(True)
        self._slider.setValue(tick)
        self._slider.blockSignals(False)
        self.valueChanged.emit(v)
        self._timer.start()

    def _emit_command(self) -> None:
        self._timer.stop()
        self.commandReady.emit(self.command())
