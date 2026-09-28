# -*- coding: utf-8 -*-
"""
openmv_viewer.py —— PC 端：开窗口实时查看 OpenMV 画面
================================================================================
配合 OpenMV 端 openmv_stream.py 使用：相机把 JPEG 通过 USB 串口推流，
本程序解析帧头后在电脑上开一个 OpenCV 窗口实时显示，方便调试。

用法：
    # 1) 先把 pc_view/openmv_stream.py 部署为 OpenMV 的 main.py（用 raw REPL 或 mpremote）
    # 2) 再在本机运行：
    python openmv_viewer.py --port COM5 --start
    # --start 会先给相机发 Ctrl-C 并执行 main.py(启动推流)，不加则假定相机已在推流。
    # 快捷键：q 退出，s 保存当前帧到 jpg，+/- 缩放

依赖：pyserial, opencv-python(或 opencv-contrib), numpy
================================================================================
"""

import sys
import time
import struct
import argparse
import subprocess

import serial
import numpy as np
import cv2

MAGIC0 = 0xA5
MAGIC1 = 0x5A
MAX_JPEG = 500 * 1024      # 单帧上限，防止长度解析错导致超大分配


def open_port(port, baud, retry_s=40):
    """打开串口，容忍相机复位/重枚举导致端口短暂消失。"""
    t0 = time.time()
    while time.time() - t0 < retry_s:
        try:
            return serial.Serial(port, baud, timeout=1)
        except Exception:
            time.sleep(0.5)
    raise RuntimeError("cannot open port %s" % port)


def start_stream(port):
    """用 mpremote 复位相机，使其开机自动运行根目录的 main.py(推流脚本)。

    注意：这台 OpenMV 的固件不会在普通 Ctrl-D 后自动跑 main.py，
    必须用 `mpremote reset`（进入 raw REPL 后复位）才可靠自启。
    """
    subprocess.run([sys.executable, "-m", "mpremote", "connect", port, "reset"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2.5)   # 等摄像头重启 + 传感器稳定


def read_exact(ser, n):
    """从串口精确读取 n 字节，超时返回 None。"""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return bytes(buf)


def parse_and_display(ser, win, scale):
    """主循环：批量读入缓冲，从缓冲里找帧头 -> 长度 -> JPEG -> 解码显示。"""
    frame_cnt = 0
    fps = 0.0
    t_fps = time.time()
    bytes_total = 0
    buf = bytearray()
    magic = bytes([MAGIC0, MAGIC1])

    while True:
        # 1) 批量读入(一次性把串口里现有的都读出来，避免逐字节卡顿)
        chunk = ser.read(4096)
        if chunk:
            buf += chunk
        elif not buf:
            continue

        # 2) 从缓冲里尽量多解出完整帧
        while True:
            idx = buf.find(magic)
            if idx < 0:
                # 没有帧头：只保留可能的半个帧头(结尾的 0xA5)
                if buf and buf[-1] == MAGIC0:
                    buf = buf[-1:]
                else:
                    buf = bytearray()
                break

            # 丢弃帧头之前的内容
            if idx > 0:
                buf = buf[idx:]

            # 长度字段还没收全
            if len(buf) < 6:
                break
            n = struct.unpack("<I", buf[2:6])[0]
            if n == 0 or n > MAX_JPEG:      # 长度非法：跳过这个帧头继续找
                buf = buf[2:]
                continue

            # 整帧还没收全
            if len(buf) < 6 + n:
                break

            # 取出 JPEG
            data = bytes(buf[6:6 + n])
            buf = buf[6 + n:]
            bytes_total += n + 6

            # 解码
            arr = np.frombuffer(data, dtype=np.uint8)
            img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if img is None:
                continue

            # 统计 FPS
            frame_cnt += 1
            now = time.time()
            if now - t_fps >= 0.5:
                fps = frame_cnt / (now - t_fps)
                frame_cnt = 0
                t_fps = now
                print("FPS=%.1f  jpg=%dB  total=%.1fKB" % (fps, n, bytes_total / 1024.0))

            # 叠加 FPS 文字 + 缩放 + 显示
            cv2.putText(img, "%.1f fps" % fps, (6, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
            if scale != 1.0:
                img = cv2.resize(img, None, fx=scale, fy=scale,
                                 interpolation=cv2.INTER_NEAREST)
            cv2.imshow(win, img)

            # 按键
            k = cv2.waitKey(1) & 0xFF
            if k == ord('q'):
                return
            elif k == ord('s'):
                fn = "openmv_snap_%d.jpg" % int(time.time())
                cv2.imwrite(fn, img)
                print("saved:", fn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM5", help="OpenMV 串口(默认 COM5)")
    ap.add_argument("--baud", type=int, default=115200, help="波特率(USB VCP 忽略)")
    ap.add_argument("--scale", type=float, default=2.0, help="显示缩放(默认2)")
    ap.add_argument("--start", action="store_true", help="先启动相机 main.py 再显示")
    args = ap.parse_args()

    if args.start:
        print("resetting camera (mpremote reset) to run main.py ...")
        start_stream(args.port)

    ser = open_port(args.port, args.baud)
    win = "OpenMV Live - %s" % args.port
    print("port open:", args.port, "| q=quit s=save")
    try:
        parse_and_display(ser, win, args.scale)
    finally:
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
