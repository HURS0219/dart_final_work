# -*- coding: utf-8 -*-
"""
pc_vision_jlink.py — 海康相机绿光识别 -> 经 J-Link RTT 把坐标发给 STM32
================================================================================
链路: 海康相机(USB3) -> 本机 OpenCV 识别绿光 -> J-Link RTT 下行 -> STM32 -> yaw 自瞄
      (无 ESP32、无串口线; 复用已有的 SWD 连接)

为什么用 RTT 而不是串口:
  现场只有 J-Link/SWD, 没有空闲串口。固件 app/link 已把 RTT 下行通道(0)接入
  命令解析链路, 与 USART6 共用同一套语义 —— 因此 "C,x,center\n" 走 RTT 即可。

协议(与固件 app/link.c 一致):
  发送 "C,<x>,<center>\\n"   x=绿光中心像素, center=画面中心像素
  固件据此发布 vision_cmd -> app/vision -> aim_cmd -> app/yaw

依赖: pip install numpy opencv-python
      海康相机需装 MVS (本脚本内置默认 MvImport 路径)
      J-Link 需装 SEGGER J-Link 软件(用 JLink.exe 写内存)

用法:
  python pc_vision_jlink.py                 # 自动找海康相机, 默认参数
  python pc_vision_jlink.py --test          # 无相机: 发送模拟摆动坐标(验证链路)
  python pc_vision_jlink.py --no-show       # 不显示窗口
  python pc_vision_jlink.py --hz 30 --center 720
"""

import argparse
import os
import subprocess
import sys
import tempfile
import time

import numpy as np

# ---- 海康 MVS SDK 路径(按本机安装位置) ----
MVS_IMPORT = r"C:\Program Files (x86)\MVS\Development\Samples\Python\MvImport"

# ---- J-Link 与固件 ELF ----
JLINK = r"C:\Program Files\SEGGER\JLink_V966\JLink.exe"
NM = r"D:\workp\tea\pack\Toolchain\arm_gnu_toolchain\bin\arm-none-eabi-nm.exe"
ELF = r"D:\worka\dart_final_work\make_one\build_dart_launcher_final\control-2026.elf"
DEVICE = "STM32F407IG"

# ---- 绿色阈值 (Hikvision, BGR->HSV; 按现场光照标定) ----
HSV_LO = (56, 120, 60)
HSV_HI = (82, 255, 255)
MIN_AREA = 20


# ============================================================================
#                          J-Link RTT 下行发送
# ============================================================================
class RttSender(object):
    """通过 J-Link 直接写固件的 RTT 下行缓冲, 把一行命令喂给 STM32。

    原理(与 Tools/scripts/rtt_send.ps1 一致):
      1) 从 ELF 解析 _acDownBuffer / _SEGGER_RTT 地址(每次重编都会变, 故动态解析);
      2) 计算 aDown[0] 的 WrOff/RdOff 在控制块内的偏移;
      3) 用 JLink.exe 写缓冲字节 -> 清 RdOff(若满) -> 写 WrOff 触发固件读取。
    """

    DOWN_BUF_MAX = 16          # SEGGER_RTT_Conf.h: BUFFER_SIZE_DOWN = 16
    MAX_NUM_UP = 3             # SEGGER_RTT_Conf.h: SEGGER_RTT_MAX_NUM_UP_BUFFERS = 3

    def __init__(self, elf=ELF, jlink=JLINK, nm=NM, device=DEVICE):
        self.elf = elf
        self.jlink = jlink
        self.nm = nm
        self.device = device
        self.buf = self._sym("_acDownBuffer")
        self.cb = self._sym("_SEGGER_RTT")
        if self.buf is None or self.cb is None:
            raise RuntimeError("解析 RTT 符号失败, 检查 ELF: %s" % elf)
        # 控制块布局: acID[16] + MaxUp(4) + MaxDown(4) = 24; 每个 up buffer 24B;
        # down buffer 内 WrOff 偏移 12, RdOff = WrOff + 4
        wr_ofs = 24 + self.MAX_NUM_UP * 24 + 12
        self.wr = self.cb + wr_ofs
        self.rd = self.wr + 4
        # 复用同一个 jlink 脚本模板, 减少进程启动开销
        self._script_path = os.path.join(tempfile.gettempdir(), "pc_vision_rtt.jlink")

    def _sym(self, name):
        try:
            out = subprocess.run([self.nm, self.elf], capture_output=True, text=True, timeout=15).stdout
        except Exception:
            return None
        for line in out.splitlines():
            tok = line.split()
            if len(tok) >= 3 and tok[-1] == name:
                return int(tok[0], 16)
        return None

    def send(self, cmd):
        """发送一行命令(自动补 \\n)。命令过长(> DOWN_BUF_MAX-1)会被截断保护。"""
        data = (cmd + "\n").encode("ascii", "ignore")
        if len(data) > self.DOWN_BUF_MAX:
            return False
        # 4 字节对齐写入(用 w4 一次写 4 字节)
        pad = data + b"\x00" * ((4 - len(data) % 4) % 4)
        lines = ["si SWD", "speed 4000", "device %s" % self.device, "connect", "h"]
        for i in range(0, len(pad), 4):
            w = int.from_bytes(pad[i:i + 4], "little")
            lines.append("w4 0x%08X 0x%08X" % (self.buf + i, w))
        lines.append("w4 0x%08X 0x00000000" % self.rd)      # 清 RdOff(防溢出)
        lines.append("w4 0x%08X 0x%08X" % (self.wr, len(data)))  # 置 WrOff 触发
        lines.append("g")
        lines.append("qc")
        try:
            with open(self._script_path, "w") as f:
                f.write("\n".join(lines))
            subprocess.run([self.jlink, "-NoGui", "1", "-CommanderScript", self._script_path],
                           capture_output=True, timeout=20)
            return True
        except Exception:
            return False


# ============================================================================
#                              相机封装
# ============================================================================
class HikCamera(object):
    """海康 MVS SDK (USB3/GigE)"""

    def __init__(self):
        sys.path.insert(0, MVS_IMPORT)
        global MvCamera, MV_CC_DEVICE_INFO_LIST, MV_USB_DEVICE, MV_GIGE_DEVICE, MV_CC_DEVICE_INFO, cast, POINTER
        from MvCameraControl_class import (MvCamera, MV_CC_DEVICE_INFO_LIST, MV_USB_DEVICE,
                                           MV_GIGE_DEVICE, MV_CC_DEVICE_INFO)
        from ctypes import cast, POINTER
        self.cam = None
        self._payload = None
        self._buf = None

    def open(self):
        dev_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, dev_list)
        if ret != 0 or dev_list.nDeviceNum == 0:
            return False
        self.cam = MvCamera()
        info = cast(dev_list.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents
        if self.cam.MV_CC_CreateHandle(info) != 0:
            return False
        if self.cam.MV_CC_OpenDevice() != 0:
            return False
        # 关闭触发, 连续采集
        self.cam.MV_CC_SetEnumValue("TriggerMode", 0)
        if self.cam.MV_CC_StartGrabbing() != 0:
            return False
        return True

    def read(self):
        from MvCameraControl_class import MV_FRAME_OUT_INFO_EX
        from ctypes import memset, byref, sizeof, c_ubyte, POINTER as P
        if self._payload is None:
            st = self.cam.MV_CC_GetIntValue("PayloadSize")
            self._payload = st.nCurValue if hasattr(st, "nCurValue") else 1920 * 1200 * 3
            self._buf = (c_ubyte * self._payload)()
        info = MV_FRAME_OUT_INFO_EX()
        memset(byref(info), 0, sizeof(info))
        ret = self.cam.MV_CC_GetOneFrameTimeout(self._buf, self._payload, info, 1000)
        if ret != 0:
            return None
        w, h = info.nWidth, info.nHeight
        arr = np.frombuffer(self._buf, dtype=np.uint8, count=w * h * 3).reshape(h, w, 3)
        return arr.copy()

    def close(self):
        try:
            if self.cam:
                self.cam.MV_CC_StopGrabbing()
                self.cam.MV_CC_CloseDevice()
                self.cam.MV_CC_DestroyHandle()
        except Exception:
            pass


def detect_green(bgr):
    """返回 (cx, cy, mask); 未识别返回 (None, None, mask)"""
    import cv2
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(HSV_LO), np.array(HSV_HI))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return None, None, mask
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < MIN_AREA:
        return None, None, mask
    m = cv2.moments(c)
    if m["m00"] == 0:
        return None, None, mask
    return int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"]), mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--hz", type=int, default=30, help="发送频率")
    ap.add_argument("--center", type=int, default=-1, help="画面中心 x; <0 自动取宽度中心")
    ap.add_argument("--test", action="store_true", help="无相机: 发模拟摆动坐标验证链路")
    ap.add_argument("--no-show", action="store_true", help="不显示窗口")
    ap.add_argument("--elf", default=ELF, help="固件 ELF(用于解析 RTT 符号)")
    args = ap.parse_args()

    print("== PC 视觉 -> J-Link RTT -> STM32 ==")
    try:
        rtt = RttSender(elf=args.elf)
    except Exception as e:
        print("RTT 初始化失败:", e)
        return 1
    print("RTT 就绪: down_buf=0x%08X WrOff=0x%08X RdOff=0x%08X" % (rtt.buf, rtt.wr, rtt.rd))

    # 让 yaw 进入自瞄模式
    rtt.send("Y,1")
    time.sleep(0.1)
    print("已发送 Y,1 (yaw 自瞄模式)")

    period = 1.0 / max(1, args.hz)
    center = args.center
    cam = None
    import cv2

    if args.test:
        print("== 测试模式: 发送模拟摆动坐标 ==")
        t0 = time.time()
        while True:
            dt = time.time() - t0
            x = int(360 + 360 * np.sin(dt * 1.5))     # 在 0..720 摆动
            c = 720
            ok = rtt.send("C,%d,%d" % (x, c))
            print("  send C,%d,%d  %s" % (x, c, "OK" if ok else "FAIL"))
            time.sleep(period)
    else:
        cam = HikCamera()
        if not cam.open():
            print("相机打开失败; 可加 --test 用模拟数据验证链路")
            return 1
        print("相机已打开")
        smooth = None
        last = 0.0
        while True:
            frame = cam.read()
            if frame is None:
                continue
            if center < 0:
                center = frame.shape[1] // 2
            x, y, mask = detect_green(frame)
            found = x is not None
            if found:
                smooth = x if smooth is None else (0.7 * smooth + 0.3 * x)
                x_send = int(round(smooth))
            else:
                x_send = 0      # 0 = 未识别(固件侧 found=0)
            now = time.time()
            if now - last >= period:
                last = now
                ok = rtt.send("C,%d,%d" % (x_send, center))
                print("  x=%4d center=%4d found=%d  %s" % (x_send, center, int(found), "OK" if ok else "FAIL"))
            if not args.no_show:
                vis = frame.copy()
                if found:
                    cv2.circle(vis, (x, y), 12, (0, 0, 255), 2)
                    cv2.line(vis, (center, 0), (center, vis.shape[0]), (255, 0, 0), 1)
                cv2.imshow("green", vis)
                cv2.imshow("mask", mask)
                if cv2.waitKey(1) & 0xFF == 27:
                    break
    if cam:
        cam.close()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
