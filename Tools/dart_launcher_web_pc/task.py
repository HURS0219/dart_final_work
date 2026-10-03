# task.py — 自动化发射任务对象 (调用封装好的状态机, 逐步执行)


class Task:
    STEPS = ["空闲", "舵机→标准位", "拉簧→标准位", "拉簧→预备位",
             "舵机→预备位", "拉簧→复位", "舵机→复位", "完成"]

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

    def set_turns(self, t):
        self.bus.send("W,%d" % int(round(t * 100)))

    def set_yaw(self, mode):
        self.bus.send("Y,%d" % mode)

    def step(self):
        s = self.bus.state
        return s["task"]["step"] if s else 0

    def step_name(self):
        return self.STEPS[self.step()] if self.step() < len(self.STEPS) else "?"
