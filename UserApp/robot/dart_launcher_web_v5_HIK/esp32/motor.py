# motor.py — 电机对象 (基类 Motor + M3508 / M2006 / GM6020 子类)
# 每个子类封装各自的类型码/减速比/单位换算, 上层只用统一接口。

from proto import MODE_STOP, MODE_SPEED, MODE_ANGLE, MODE_TURNS


class Motor:
    TYPE_NAME = "MOTOR"
    TYPE_CODE = -1
    GEAR_RATIO = 1.0

    def __init__(self, bus, slot, role, can_id, reverse=False):
        self.bus = bus
        self.slot = slot
        self.role = role
        self.can_id = can_id
        self.reverse = reverse

    # ---- 控制 ----
    def stop(self):
        self.bus.send("M,%d,%d,0" % (self.slot, MODE_STOP))

    def speed(self, rpm):
        self.bus.send("M,%d,%d,%d" % (self.slot, MODE_SPEED, int(round(rpm * 10))))

    def angle(self, deg):
        self.bus.send("M,%d,%d,%d" % (self.slot, MODE_ANGLE, int(round(deg * 10))))

    def turns(self, t):
        self.bus.send("M,%d,%d,%d" % (self.slot, MODE_TURNS, int(round(t * 100))))

    # ---- 零点 / 方向 ----
    def zero(self):
        self.bus.send("Z,%d" % self.slot)

    def toggle_dir(self):
        self.bus.send("D,%d" % self.slot)
        self.reverse = not self.reverse

    # ---- 参数 ----
    def set_param(self, pid, value):
        self.bus.send("P,%d,%d,%d" % (self.slot, pid, value))

    def set_params(self, items):
        # items: [(pid, raw_value), ...]
        for pid, val in items:
            self.bus.send("P,%d,%d,%d" % (self.slot, pid, val))

    def reset_params(self):
        self.bus.send("R,%d" % self.slot)

    # ---- 状态 ----
    def state(self):
        return self.bus.motor(self.slot)

    def params(self):
        return self.bus.params(self.slot)

    def online(self):
        m = self.state()
        return bool(m and m["online"])

    def rpm(self):
        m = self.state()
        return m["rpm"] if m else 0

    def angle_deg(self):
        m = self.state()
        return m["angle100"] / 100.0 if m else 0.0

    def turns_now(self):
        m = self.state()
        return m["turns100"] / 100.0 if m else 0.0


class M3508(Motor):
    TYPE_NAME = "M3508"
    TYPE_CODE = 1
    GEAR_RATIO = 19.2032


class M2006(Motor):
    TYPE_NAME = "M2006"
    TYPE_CODE = 2
    GEAR_RATIO = 36.0


class GM6020(Motor):
    TYPE_NAME = "GM6020"
    TYPE_CODE = 0
    GEAR_RATIO = 1.0


def make_motor(bus, slot, role, can_id, type_code, reverse=False):
    cls = {0: GM6020, 1: M3508, 2: M2006}.get(type_code, Motor)
    return cls(bus, slot, role, can_id, reverse)
