# main.py — 制导飞镖发射架 · 网页控制台 (ESP32-S3-CAM / MicroPython)
#   配套固件: UserApp/robot/dart_launcher_final  (v1.0.7+)
#
# 组装: 总线驱动 Bus + 电机/舵机/任务对象 + HTTP 服务
#
# ── 接线(务必核对) ────────────────────────────────────────────────
#   ESP32-S3-CAM              C 板 (STM32F407)
#     GPIO4  (TX) ──────────> PG9   (USART6_RX)
#     GPIO5  (RX) <────────── PG14  (USART6_TX)
#     GND         ──────────  GND
#     OTG 口      ──────────  PC(USB)   ← 必须插 OTG 口(原生USB)
#
#   波特率 115200 8N1(两端一致)。C 板侧为 LAUNCH_LINK_UART = &huart6。
#
#   ⚠ USB 必须插板上 "OTG" 口: 本脚本用 sys.stdin 做 PC<->C板 双向桥,
#     只有原生 USB(GPIO19/20) 才有数据; 插 "UART" 口(CH343) 桥会失效。
#   ⚠ 勿占用: GPIO26~32(Flash), GPIO35/36/37(PSRAM), GPIO19/20(USB),
#             GPIO0/45/46(strapping)。GPIO17/18 是 CAM 预留, 本工程不用。
#
# 依赖同目录: proto.py motor.py servo.py task.py web.py 以及 www/ 静态文件。

import network
import select
import sys
import time

from proto import Bus
from motor import M3508
from servo import PTK7350
from task import Task
from web import WebServer

AP_SSID = "DART_CTRL"
AP_PASS = "12345678"

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=AP_SSID, password=AP_PASS, max_clients=4)
while not ap.active():
    time.sleep_ms(50)
print("AP ready:", ap.ifconfig()[0])

# UART1 接 C 板: GPIO4=TX, GPIO5=RX(见文件头接线表; 默认值即为此, 显式写出便于核对)
bus = Bus(uart_id=1, tx=4, rx=5, baud=115200)
# slot 必须与固件 Launcher_MotorSlot_e 一致: 0=拉簧A 1=拉簧B 2=丝杆
# (yaw 已独立成 app, 走 Y 命令 + task.yaw 字段, 不在此列表)
motors = [
    M3508(bus, 0, "拉簧A", 2),
    M3508(bus, 1, "拉簧B", 3),
    M3508(bus, 2, "丝杆", 4),
]
servo = PTK7350(bus, "trigger")
task = Task(bus)
web = WebServer(bus, motors, servo, task)
print("HTTP server listening on :80")

# ---- USB 串口(PC) 接收 ----
_poll = select.poll()
_poll.register(sys.stdin, select.POLLIN)
_usb_buf = ""


def usb_rx():
    """读取 PC 经 USB 发来的整行, 转发给 C 板"""
    global _usb_buf
    while True:
        try:
            if not _poll.poll(0):
                break
        except Exception:
            break
        ch = sys.stdin.read(1)
        if not ch:
            break
        if ch == "\n" or ch == "\r":
            line = _usb_buf.strip()
            _usb_buf = ""
            if line:
                bus.send(line)  # 例如 "C,720,720" 或 "M,3,1,100"
        else:
            _usb_buf += ch
            if len(_usb_buf) > 64:
                _usb_buf = ""


last_hb = 0
last_dbg = 0
while True:
    bus.poll()
    usb_rx()
    web.poll_http()
    now = time.ticks_ms()
    if time.ticks_diff(now, last_hb) > 500:
        last_hb = now
        bus.send("H")
    # 短调试行(2Hz), 供上位机/串口观察; 数据量很小不会撑爆 USB
    if bus.state and time.ticks_diff(now, last_dbg) > 500:
        last_dbg = now
        try:
            st = bus.state
            # 电机只到 slot 2(3 路); yaw 无独立 slot, 用 task.yaw 观察模式
            print("D yaw=%s aim=%s vis=%s,%s e=%s rpmA=%s" %
                  (st["task"]["yaw"], st["task"]["aim100"], st["vis"]["ok"],
                   st["vis"]["x"], st["vis"]["err"], st["motors"][0]["rpm"]))
        except Exception:
            pass
    time.sleep_ms(3)
