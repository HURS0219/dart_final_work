# task.py — 自动化发射任务对象 (调用封装好的状态机, 逐步执行)
#
# 【v1.0.7 变更】上膛目标由"圈"改为"角度", 且拉簧 A/B 可各自独立设定:
#   - 旧: W,<turns100>            (圈×100, 无 slot, A/B 强制同值)  -> 固件已废弃, 回 ERR
#   - 新: N,<slot>,<deg>          (整数角度, 逐路独立)
#   本模块保留旧的兼容接口名 set_turns(), 但内部改为发角度 + 逐路。


class Task:
    STEPS = ["空闲", "舵机→标准位", "拉簧→标准位", "拉簧→预备位",
             "舵机→预备位", "拉簧→复位", "舵机→复位", "完成"]

    # 电机 slot 定义(与固件 Launcher_MotorSlot_e 一致)
    SLOT_SPRING_A = 0
    SLOT_SPRING_B = 1
    SLOT_SCREW = 2

    def __init__(self, bus):
        self.bus = bus

    def start(self):
        self.bus.send("G,10")

    def stop(self):
        self.bus.send("G,11")

    def estop(self):
        self.bus.send("G,12")

    def clear_estop(self):
        self.bus.send("G,13")

    def spring_std(self):
        self.bus.send("G,0")

    def spring_prep(self):
        self.bus.send("G,1")

    def servo_std(self):
        self.bus.send("G,2")

    def servo_prep(self):
        self.bus.send("G,3")

    # ---- 上膛角度设定(角度制, 逐路独立) ----
    def set_spring_deg(self, deg, slot=None):
        """设定拉簧上膛角度(整数 deg, 输出侧)。
        slot=None 时同时设 A 和 B(保持旧行为); 指定 slot 则只设该路。"""
        d = int(round(deg))
        if slot is None:
            for s in (self.SLOT_SPRING_A, self.SLOT_SPRING_B):
                self.bus.send("N,%d,%d" % (s, d))
        else:
            self.bus.send("N,%d,%d" % (slot, d))

    def set_spring_deg_ab(self, deg_a, deg_b):
        """A/B 分别设定上膛角度(两根弹簧行程可不同)。"""
        self.bus.send("N,%d,%d" % (self.SLOT_SPRING_A, int(round(deg_a))))
        self.bus.send("N,%d,%d" % (self.SLOT_SPRING_B, int(round(deg_b))))

    def set_screw_deg(self, deg):
        """设定丝杆目标角度(整数 deg; 自锁丝杆到位后会自行卸力)。"""
        self.bus.send("N,%d,%d" % (self.SLOT_SCREW, int(round(deg))))

    def set_yaw(self, mode):
        self.bus.send("Y,%d" % mode)

    def step(self):
        s = self.bus.state
        return s["task"]["step"] if s else 0

    def step_name(self):
        return self.STEPS[self.step()] if self.step() < len(self.STEPS) else "?"

    # ---- 状态读取 ----
    def spring_deg(self):
        """A 拉簧当前预备位设定(deg)。"""
        s = self.bus.state
        return s["task"]["spring_a_deg"] if s else 0

    def estop_on(self):
        s = self.bus.state
        return bool(s and s["task"]["estop"])
