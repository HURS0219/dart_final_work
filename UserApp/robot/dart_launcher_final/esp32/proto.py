# proto.py — 控制总线驱动 (ESP32 <-> C板 UART 协议)
# 每个网络对象的底层通信都通过本类。一旦测好即冻结, 上层只调接口。
#
# 【v1.0.7 协议变更 — 与 dart_launcher_final 固件配套】
#   1) 角度制: 全工程不再用"圈"。遥测里原 turns100(圈×100) 改为 deg100(角度×100);
#      原 spring100(圈×100) 改为 spring_a_deg(A 拉簧预备位角度)。
#      字段**个数与顺序不变**, 故本文件的解析逻辑无需改动, 只是语义变了。
#   2) 电机路数 4 -> 3: yaw 已独立成 app, 不再出现在 motor_fb 里,
#      其状态改从 task 段的 yaw100/yawang 读到(见 _parse_state 的 "yaw" 字段)。
#   3) 设定上膛角度不再用 W(圈), 改用 N,<slot>,<deg>(整数角度), 见 motor.py。

from machine import UART

MODE_STOP = 0
MODE_SPEED = 1
MODE_ANGLE = 2
# MODE_TURNS 已废弃: 固件改为角度制后不再支持"圈", 保留常量仅为兼容旧引用
MODE_TURNS = 3

PARAM_ORDER = [1, 2, 3, 4, 5, 7, 8, 10, 11, 12, 6]  # 与 C 端一致


class Bus:
    def __init__(self, uart_id=1, tx=17, rx=18, baud=115200):
        self.uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx, rxbuf=1024)
        self.rxbuf = b""
        self.state = None      # 解析后的遥测
        self.raw = ""          # 最近一帧原始遥测
        self.scan_ids = []
        self.pong = False
        self.saved = False
        self._t = 0

    # ---- 发送 ----
    def send(self, s):
        try:
            self.uart.write(s + "\n")
        except Exception:
            pass

    def ping(self):
        self.send("PING")

    def scan(self):
        self.send("S")

    def save(self):
        self.send("SAVE")

    def inject_vision(self, x, center):
        self.send("C,%d,%d" % (x, center))

    # ---- 接收 ----
    def poll(self):
        d = self.uart.read()
        if d:
            self.rxbuf += d
        while b"\n" in self.rxbuf:
            line, self.rxbuf = self.rxbuf.split(b"\n", 1)
            self._handle(line.strip())

    def _handle(self, line):
        if line.startswith(b"F,"):
            st = _parse_state(line)
            if st:
                self.state = st
                self.raw = line.decode()
        elif line.startswith(b"S,"):
            p = line.decode().split(",")
            if len(p) >= 2:
                n = int(p[1])
                self.scan_ids = [int(x) for x in p[2:2 + n]]
        elif line.startswith(b"PONG"):
            self.pong = True
        elif line.startswith(b"SAVED"):
            self.saved = True

    def motor(self, slot):
        if self.state and slot < len(self.state["motors"]):
            return self.state["motors"][slot]
        return None

    def params(self, slot):
        if self.state and slot < len(self.state["params"]):
            return self.state["params"][slot]
        return None


def _nums(b):
    out = []
    cur = ""
    for c in b:
        if 48 <= c <= 57 or c == 45:  # 0-9 or '-'
            cur += chr(c)
        else:
            if cur:
                out.append(int(cur))
                cur = ""
    if cur:
        out.append(int(cur))
    return out


def _parse_state(line):
    # 解析遥测帧 "F,..."
    # 【字段顺序与 v1.0.6 完全一致, 仅语义变化(圈->角度), 故不会错位】
    #   F,<n>,
    #     每电机 12 项 × n : slot,type,id,online,dir,mode,target100,
    #                       rpm,angle100,deg100,temp,cur
    #     舵机 4 项        : cur10,state,std10,prep10
    #     任务 5 项        : spring_a_deg,step,yaw,estop,aim100
    #     视觉 4 项        : x,ok,center,err
    #     每电机 11 参数 × n
    s = line.decode()
    v = _nums(s.encode())
    # v[0] = n
    if len(v) < 1:
        return None
    n = v[0]
    i = 1
    motors = []
    for _ in range(n):
        if i + 12 > len(v):
            return None
        motors.append({
            "slot": v[i], "type": v[i + 1], "id": v[i + 2], "online": v[i + 3],
            "dir": v[i + 4], "mode": v[i + 5], "target100": v[i + 6], "rpm": v[i + 7],
            "angle100": v[i + 8],
            # 原 turns100(圈×100) -> 现 deg100(角度×100)。保留 turns100 键名做兼容别名,
            # 但值已是角度, 新代码请用 deg100。
            "deg100": v[i + 9], "turns100": v[i + 9],
            "temp": v[i + 10], "cur": v[i + 11],
        })
        i += 12
    if i + 4 > len(v):
        return None
    servo = {"cur10": v[i], "state": v[i + 1], "std10": v[i + 2], "prep10": v[i + 3]}
    i += 4
    # task[0] 原 spring100(圈×100) -> 现 spring_a_deg(A 拉簧预备位角度, 整数)
    task = {"spring_a_deg": v[i], "spring100": v[i], "step": v[i + 1],
            "yaw": v[i + 2], "estop": v[i + 3], "aim100": v[i + 4]}
    i += 5
    vis = {"x": v[i], "ok": v[i + 1], "center": v[i + 2], "err": v[i + 3]}
    i += 4
    params = []
    for _ in range(n):
        if i + 11 > len(v):
            break
        params.append(v[i:i + 11])
        i += 11
    return {"n": n, "motors": motors, "servo": servo, "task": task, "vis": vis, "params": params}
