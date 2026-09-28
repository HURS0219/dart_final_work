# task.py — 状态机 / 总控对象  v0.2
# 模式: 0 待机 / 1 手动 / 2 混控 / 3 自检
# 混控指令按"百分比"下发 (-100 ~ 100)，内部转成协议值 (x10)。


class Task:
    MODES = ["待机", "手动", "混控", "自检"]

    def __init__(self, bus):
        self.bus = bus

    def set_mode(self, m):
        self.bus.send("MODE,%d" % int(m))

    def idle(self):
        self.set_mode(0)

    def manual(self):
        self.set_mode(1)

    def mix(self):
        self.set_mode(2)

    def test(self):
        self.set_mode(3)

    def set_mix(self, p_pct, y_pct, r_pct):
        self.bus.send("MIX,%d,%d,%d" % (int(round(p_pct * 10)),
                                        int(round(y_pct * 10)),
                                        int(round(r_pct * 10))))

    def save(self):
        self.bus.send("SAVE")

    def heartbeat(self):
        self.bus.send("H")

    def ping(self):
        self.bus.send("PING")

    # ---- 遥测 ----
    def mode(self):
        s = self.bus.state
        return s["mode"] if s else 0

    def mode_name(self):
        m = self.mode()
        return self.MODES[m] if m < len(self.MODES) else "?"

    def link(self):
        s = self.bus.state
        return s["link"] if s else 0

    def mix_cmd(self):
        s = self.bus.state
        return s["mix"] if s else {"p": 0, "y": 0, "r": 0}
