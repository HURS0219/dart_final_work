# main.py — 制导飞镖舵机子系统 v0.2 · 网页控制台 (ESP32-S3 / MicroPython)
#
# 依赖同目录: bus.py servo.py task.py web.py 以及 www/ 静态文件。

import network
import time

from bus import Bus
from servo import PTK7350
from task import Task
from web import WebServer

AP_SSID = "DART_SERVO"
AP_PASS = "12345678"

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=AP_SSID, password=AP_PASS, max_clients=4)
while not ap.active():
    time.sleep_ms(50)
print("AP ready:", ap.ifconfig()[0])

bus = Bus()
servos = [
    PTK7350(bus, 0, "右上"),
    PTK7350(bus, 1, "左上"),
    PTK7350(bus, 2, "左下"),
    PTK7350(bus, 3, "右下"),
]
task = Task(bus)
web = WebServer(bus, servos, task)
print("HTTP server listening on :80")

last_hb = 0
while True:
    bus.poll()
    web.poll_http()
    now = time.ticks_ms()
    if time.ticks_diff(now, last_hb) > 200:
        last_hb = now
        task.heartbeat()
    time.sleep_ms(2)
