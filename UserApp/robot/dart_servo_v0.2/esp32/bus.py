# bus.py — 控制总线 (ESP32 <-> C板 UART 协议)  v0.2
#
# C 端遥测帧 (100ms):
#   T,<mode>,<link>,
#     <a0>,<p0>,<d0>,<t0>,<s0>, ... x4,
#     <mp>,<my>,<mr>
#   a=角度x10  p=脉宽us  d=方向  t=trim x10  s=scale x1000  m=mix x1000
#
# 一旦测好即冻结, 上层只调接口。

from machine import UART

MODE_IDLE = 0
MODE_MANUAL = 1
MODE_MIX = 2
MODE_TEST = 3


class Bus:
    def __init__(self, uart_id=1, tx=17, rx=18, baud=115200):
        self.uart = UART(uart_id, baudrate=baud, tx=tx, rx=rx, rxbuf=1024)
        self.rxbuf = b""
        self.state = None
        self.raw = ""
        self.pong = False
        self.saved = 0  # 0 无 / 1 成功 / 2 失败

    # ---- 发送 ----
    def send(self, s):
        try:
            self.uart.write(s + "\n")
        except Exception:
            pass

    # ---- 接收 ----
    def poll(self):
        d = self.uart.read()
        if d:
            self.rxbuf += d
        while b"\n" in self.rxbuf:
            line, self.rxbuf = self.rxbuf.split(b"\n", 1)
            self._handle(line.strip())

    def _handle(self, line):
        if line.startswith(b"T,"):
            st = _parse(line)
            if st:
                self.state = st
                self.raw = line.decode()
        elif line.startswith(b"PONG"):
            self.pong = True
        elif line.startswith(b"SAVED"):
            self.saved = 1
        elif line.startswith(b"SAVEERR"):
            self.saved = 2

    def axis(self, i):
        if self.state and i < len(self.state["axes"]):
            return self.state["axes"][i]
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


def _parse(line):
    v = _nums(line)
    if len(v) < 2:
        return None
    mode = v[0]
    link = v[1]
    i = 2
    if i + 20 + 3 > len(v):
        return None
    axes = []
    for _ in range(4):
        axes.append({
            "angle10": v[i], "pulse": v[i + 1], "dir": v[i + 2],
            "trim10": v[i + 3], "scale1000": v[i + 4],
        })
        i += 5
    mix = {"p": v[i], "y": v[i + 1], "r": v[i + 2]}
    return {"mode": mode, "link": link, "axes": axes, "mix": mix}
