# main.py — 制导飞镖 舵面控制台 (ESP32-S3 / MicroPython)
#
# 组装: 总线驱动 Bus + 4 路舵面 Servo + 舵面状态机 Surface + HTTP 服务
# 手机连 AP (DART_FC / 12345678), 浏览器开 http://192.168.4.1
#
# 依赖同目录: proto.py servo.py web.py 以及 www/ 静态文件。

import network
import time

from proto import Bus
from servo import PTK7350, Surface
from web import WebServer

AP_SSID = "DART_FC"
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
surface = Surface(bus)
web = WebServer(bus, servos, surface)
print("HTTP server listening on :80")

last_hb = 0
last_dbg = 0
while True:
    bus.poll()
    web.poll_http()
    now = time.ticks_ms()
    if time.ticks_diff(now, last_hb) > 500:
        last_hb = now
        bus.send("H")
    if bus.state and time.ticks_diff(now, last_dbg) > 500:
        last_dbg = now
        try:
            st = bus.state
            print("D state=%s fs=%s vis=%s d=%s" %
                  (st["state"], st["failsafe"], st["visok"], st["defl"]))
        except Exception:
            pass
    time.sleep_ms(3)
