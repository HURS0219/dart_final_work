# proto.py — 制导飞镖控制总线驱动 (ESP32 <-> C板 UART 协议)
# 底层通信只在这里实现, 一旦测好即冻结, 上层只调接口。

from machine import UART

# 舵面状态机 (与 dart_surface.h 一致)
S_BOOT, S_NEUTRAL, S_ACTIVE, S_TEST, S_MANUAL, S_FAULT = range(6)

# 参数表 (与 dart_cfg.h 一致)
P_SERVO_NEUTRAL = 0
P_SERVO_TRIM = 4
P_SERVO_MAX = 8
P_SERVO_REVERSE = 12
P_SERVO_PULSE_MIN = 16
P_SERVO_PULSE_MAX = 17
P_SERVO_RANGE = 18
P_SERVO_RATE = 19
P_MIX = 20
P_ROLL_KP = 32
P_ROLL_KI = 33
P_ROLL_KD = 34
P_PITCH_KP = 35
P_PITCH_KI = 36
P_PITCH_KD = 37
P_YAW_KP = 38
P_YAW_KI = 39
P_YAW_KD = 40
P_NAV_RATIO = 41
P_V_CLOSE = 42
P_PNG_MAX_G = 43
P_FAILSAFE_MS = 44
PARAM_CNT = 45


class Bus:
    def __init__(self, uart_id=1, tx=17, rx=18, baud=115200):
        self.uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx, rxbuf=1024)
        self.rxbuf = b""
        self.state = None      # 解析后的遥测
        self.raw = ""          # 最近一帧原始遥测
        self.pong = False
        self.saved = False

    # ---- 发送 ----
    def send(self, s):
        try:
            self.uart.write(s + "\n")
        except Exception:
            pass

    def ping(self):
        self.send("H")

    def save(self):
        self.send("SAVE")

    def inject_vision(self, x, y, w=24, h=18):
        self.send("C,%d,%d,%d,%d" % (x, y, w, h))

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
        elif line.startswith(b"PONG"):
            self.pong = True
        elif line.startswith(b"SAVED"):
            self.saved = True

    def servo(self, idx):
        if self.state and idx < len(self.state["defl"]):
            return self.state["defl"][idx]
        return 0.0

    def param(self, pid):
        if self.state and pid < len(self.state["params"]):
            return self.state["params"][pid]
        return 0


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
    v = _nums(line)
    if len(v) < 17:
        return None
    i = 0
    state = v[i]; failsafe = v[i + 1]; link = v[i + 2]; visok = v[i + 3]; i += 4
    defl = v[i:i + 4]; i += 4
    raw = v[i:i + 4]; i += 4
    mix = v[i:i + 3]; i += 3
    att = v[i:i + 3]; i += 3
    visx, visy = v[i], v[i + 1]; i += 2
    np = v[i]; i += 1
    params = v[i:i + np]
    return {"state": state, "failsafe": failsafe, "link": link, "visok": visok,
            "defl": defl, "raw": raw, "mix": mix, "att": att,
            "visx": visx, "visy": visy, "np": np, "params": params}
