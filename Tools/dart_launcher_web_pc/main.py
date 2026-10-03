# main.py — PC 端入口: J-Link(SWD/RTT) <-> 网页 桥接 (无 ESP)
#
# 运行: python main.py   然后浏览器打开 http://127.0.0.1:8000
# 依赖: pip install pylink-square  (J-Link 需用 SWD 接 C 板)

import time

from proto import Bus
from motor import M3508, M2006
from servo import PTK7350
from task import Task
from web import WebServer

def now_ms():
    return int(time.time() * 1000)


def main():
    print("opening J-Link RTT ...")
    bus = Bus()
    motors = [
        M3508(bus, 0, "拉簧A", 2),
        M3508(bus, 1, "拉簧B", 3),
        M3508(bus, 2, "扳机", 4),
        M2006(bus, 3, "Yaw", 1),
    ]
    servo = PTK7350(bus, "trigger")
    task = Task(bus)
    web = WebServer(bus, motors, servo, task)
    print("HTTP server: http://127.0.0.1:8000")
    print("(Ctrl+C 退出)")

    last_hb = 0
    last_dbg = 0
    while True:
        bus.poll()
        web.poll_http()
        now = now_ms()
        if now - last_hb > 500:
            last_hb = now
            bus.send("H")
        if bus.state and now - last_dbg > 1000:
            last_dbg = now
            try:
                st = bus.state
                print("D yaw=%s aim=%s vis=%s,%s e=%s rpm=%s" %
                      (st["task"]["yaw"], st["task"]["aim100"], st["vis"]["ok"],
                       st["vis"]["x"], st["vis"]["err"], st["motors"][3]["rpm"]))
            except Exception:
                pass
        time.sleep(0.003)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nbye")
