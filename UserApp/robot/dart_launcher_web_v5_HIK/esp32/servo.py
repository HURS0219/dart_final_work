# servo.py — 舵机对象 (基类 Servo + PTK7350 / PTK7465 子类)
# 当前硬件为 PWM 舵机 (角度制, 无反馈), 子类只区分型号/量程。


class Servo:
    TYPE_NAME = "SERVO"
    DEG_RANGE = 270.0

    def __init__(self, bus, role="trigger"):
        self.bus = bus
        self.role = role

    def set_std(self, deg):
        self.bus.send("V,0,%d" % int(round(deg * 10)))

    def set_prep(self, deg):
        self.bus.send("V,1,%d" % int(round(deg * 10)))

    def go_std(self):
        self.bus.send("V,2")

    def go_prep(self):
        self.bus.send("V,3")

    def go_deg(self, deg):
        self.bus.send("V,4,%d" % int(round(deg * 10)))

    def zero(self):
        self.bus.send("V,5")

    def state(self):
        return self.bus.state["servo"] if self.bus.state else None

    def cur_deg(self):
        s = self.state()
        return s["cur10"] / 10.0 if s else 0.0


class PTK7350(Servo):
    TYPE_NAME = "PTK7350"
    DEG_RANGE = 270.0


class PTK7465(Servo):
    TYPE_NAME = "PTK7465"
    DEG_RANGE = 270.0
