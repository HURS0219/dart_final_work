# servo.py — 舵面对象 (基类 Servo + PTK7350 子类) 与 舵面状态机对象 Surface

from proto import (P_SERVO_NEUTRAL, P_SERVO_TRIM, P_SERVO_MAX, P_SERVO_REVERSE)


class Servo:
    """一路舵面: 对应 dart_surface 的第 idx 路。"""
    TYPE_NAME = "SERVO"

    def __init__(self, bus, idx, name):
        self.bus = bus
        self.idx = idx
        self.name = name

    # ---- 标定 (写入参数表, 掉电保存) ----
    def set_neutral(self, deg):
        self.bus.send("P,%d,%d" % (P_SERVO_NEUTRAL + self.idx, int(round(deg * 100))))

    def set_trim(self, deg):
        self.bus.send("P,%d,%d" % (P_SERVO_TRIM + self.idx, int(round(deg * 100))))

    def set_max(self, deg):
        self.bus.send("P,%d,%d" % (P_SERVO_MAX + self.idx, int(round(deg * 100))))

    def set_reverse(self, rev):
        self.bus.send("P,%d,%d" % (P_SERVO_REVERSE + self.idx, int(round(rev * 100))))

    # ---- 动作 ----
    def manual(self, deg):
        """MANUAL 状态下手动给逻辑偏角 (deg)"""
        self.bus.send("V,%d,%d" % (self.idx, int(round(deg * 10))))

    def zero(self):
        """取零点: 当前位置设为中立"""
        self.bus.send("Z,%d" % self.idx)


class PTK7350(Servo):
    TYPE_NAME = "PTK7350"


class PTK7465(Servo):
    TYPE_NAME = "PTK7465"


class Surface:
    """4 路舵面总状态机 + 混控 + 参数。"""

    def __init__(self, bus):
        self.bus = bus

    def set_state(self, st):
        self.bus.send("T,%d" % int(st))

    def set_mix(self, pitch, yaw, roll):
        """归一化混控 (-1..1)"""
        self.bus.send("X,%d,%d,%d" % (int(round(pitch * 100)), int(round(yaw * 100)),
                                      int(round(roll * 100))))

    def set_param(self, pid, val):
        self.bus.send("P,%d,%d" % (int(pid), int(round(val * 100))))

    def reset(self):
        self.bus.send("R")

    def save(self):
        self.bus.send("SAVE")
