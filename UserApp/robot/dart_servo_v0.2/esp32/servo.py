# servo.py — 舵面对象 (基类 Servo + PTK7350 / PTK7465 子类)  v0.2
# 命名: pwm1=右上, pwm2=左上, pwm3=左下, pwm4=右下。
# 角度单位: 度 (逻辑角 -139.5 ~ +139.5)。


class Servo:
    TYPE_NAME = "SERVO"

    def __init__(self, bus, idx, name="servo"):
        self.bus = bus
        self.idx = idx
        self.name = name

    # ---- 指令 ----
    def set_angle(self, deg):
        self.bus.send("ANG,%d,%d" % (self.idx, int(round(deg * 10))))

    def zero(self):
        self.bus.send("ZERO,%d" % self.idx)

    def toggle_dir(self):
        self.bus.send("DIR,%d" % self.idx)

    def set_trim(self, deg):
        self.bus.send("TRIM,%d,%d" % (self.idx, int(round(deg * 10))))

    def set_scale(self, scale):
        self.bus.send("SCALE,%d,%d" % (self.idx, int(round(scale * 1000))))

    def reset_cal(self):
        self.bus.send("RSTCAL,%d" % self.idx)

    # ---- 遥测 ----
    def state(self):
        return self.bus.axis(self.idx)

    def cur_deg(self):
        s = self.state()
        return s["angle10"] / 10.0 if s else 0.0


class PTK7350(Servo):
    TYPE_NAME = "PTK7350"


class PTK7465(Servo):
    TYPE_NAME = "PTK7465"
