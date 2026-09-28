# proto.py — 控制总线驱动 (ESP32 <-> C板 UART 协议)
# 每个网络对象的底层通信都通过本类。一旦测好即冻结, 上层只调接口。

from machine import UART

MODE_STOP = 0
MODE_SPEED = 1
MODE_ANGLE = 2
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
    # 去掉前导 "F," 与结尾, 解析为数字序列
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
            "angle100": v[i + 8], "turns100": v[i + 9], "temp": v[i + 10], "cur": v[i + 11],
        })
        i += 12
    if i + 4 > len(v):
        return None
    servo = {"cur10": v[i], "state": v[i + 1], "std10": v[i + 2], "prep10": v[i + 3]}
    i += 4
    task = {"spring100": v[i], "step": v[i + 1], "yaw": v[i + 2], "estop": v[i + 3], "aim100": v[i + 4]}
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
