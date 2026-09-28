# hik_vision.py — 海康工业相机自瞄上位机 (v5_HIK)
#
# 链路: 海康相机 --> 本机(OpenCV 识别绿光) --> 串口(USB, 默认 COM8) --> ESP32 --> UART1 --> C板 --> Yaw(M2006)
#       不占用 WiFi; PC 也可从同一串口读到 C 板遥测(实时调试)。
#
# 用法:
#   python hik_vision.py                 # 自动找海康相机(MVS)，找不到则回退 UVC
#   python hik_vision.py --port COM8     # 指定串口
#   python hik_vision.py --uvc 0         # 强制用 OpenCV 打开 0 号摄像头
#   python hik_vision.py --test          # 无相机: 发送模拟摆动坐标, 验证链路
#   python hik_vision.py --no-show       # 不显示窗口(纯后台)
#
# 依赖: pip install numpy opencv-python pyserial
#       海康相机还需安装 MVS (本脚本已内置默认 MvImport 路径)

import argparse
import os
import sys
import time

import numpy as np
import cv2
import serial

# ---- 海康 MVS SDK 路径 (按本机安装位置) ----
MVS_IMPORT = r"C:\Program Files (x86)\MVS\Development\Samples\Python\MvImport"

# ---- 默认参数 ----
DEF_PORT = "COM8"             # ESP32-S3 USB 串口
DEF_BAUD = 115200
DEF_SEND_HZ = 30              # 坐标发送频率
DEF_CENTER = -1               # <0 = 自动用画面宽度中心
# 绿色阈值 (OpenCV HSV: H 0~179, S/V 0~255) —— 按本机海康相机实测标定
HSV_LO = (56, 170, 70)
HSV_HI = (82, 255, 255)
MIN_AREA = 20                 # 最小绿斑面积(像素)


def detect_green(bgr):
    """返回 (cx, cy, mask); 未识别到返回 (None, None, mask)"""
    blur = cv2.GaussianBlur(bgr, (5, 5), 0)          # 去噪
    hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(HSV_LO), np.array(HSV_HI))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))  # 补洞, 边缘更稳
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


# ======================= 相机封装 =======================
class HikCamera:
    """海康 MVS SDK (GigE / USB3)"""

    def __init__(self, index=0):
        sys.path.append(MVS_IMPORT)
        from MvCameraControl_class import (  # noqa: E402
            MvCamera, MV_CC_DEVICE_INFO_LIST, MV_GIGE_DEVICE, MV_USB_DEVICE,
            MV_ACCESS_Exclusive, MV_TRIGGER_MODE_OFF, MV_FRAME_OUT_INFO_EX,
            MV_CC_PIXEL_CONVERT_PARAM, PixelType_Gvsp_BGR8_Packed, PixelType_Gvsp_RGB8_Packed,
        )
        self._M = __import__("MvCameraControl_class")
        from ctypes import byref, sizeof, memset, c_ubyte, cast, POINTER  # noqa
        self._ctypes = (byref, sizeof, memset, c_ubyte, cast, POINTER)

        dev_list = MV_CC_DEVICE_INFO_LIST()
        ret = -1
        for _ in range(10):  # 枚举偶发返回 0 台, 重试
            ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, dev_list)
            if ret == 0 and dev_list.nDeviceNum > 0:
                break
            time.sleep(0.3)
        if ret != 0 or dev_list.nDeviceNum == 0:
            raise RuntimeError("未找到海康相机 (nDeviceNum=0)")
        if index >= dev_list.nDeviceNum:
            index = 0
        self.cam = MvCamera()
        st = cast(dev_list.pDeviceInfo[index], POINTER(self._M.MV_CC_DEVICE_INFO)).contents
        self.cam.MV_CC_CreateHandle(st)
        self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        # 强制彩色像素格式(否则可能默认灰度, 无法识别绿色)
        for fmt in (self._M.PixelType_Gvsp_BayerRG8, self._M.PixelType_Gvsp_RGB8_Packed,
                    self._M.PixelType_Gvsp_BGR8_Packed):
            try:
                if self.cam.MV_CC_SetEnumValue("PixelFormat", fmt) == 0:
                    print("[HIK] PixelFormat ->", fmt)
                    break
            except Exception:
                pass
        self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        self.cam.MV_CC_StartGrabbing()

        self._MV_FRAME_OUT_INFO_EX = MV_FRAME_OUT_INFO_EX
        self._MV_CC_PIXEL_CONVERT_PARAM = MV_CC_PIXEL_CONVERT_PARAM
        self._BGR8 = PixelType_Gvsp_BGR8_Packed
        self._RGB8 = PixelType_Gvsp_RGB8_Packed
        stParam = self._M.MVCC_INTVALUE()
        self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
        self.nPayload = stParam.nCurValue
        self.buf = (c_ubyte * self.nPayload)()
        self.frame = MV_FRAME_OUT_INFO_EX()
        self.conv = (c_ubyte * (self.nPayload * 3))()
        print("[HIK] 相机已打开, payload =", self.nPayload)

    def read(self):
        byref, sizeof, memset, c_ubyte, cast, POINTER = self._ctypes
        memset(byref(self.frame), 0, sizeof(self.frame))
        ret = self.cam.MV_CC_GetOneFrameTimeout(self.buf, self.nPayload, self.frame, 1000)
        if ret != 0:
            return None
        w, h = self.frame.nWidth, self.frame.nHeight
        stConv = self._MV_CC_PIXEL_CONVERT_PARAM()
        memset(byref(stConv), 0, sizeof(stConv))
        stConv.nWidth = w
        stConv.nHeight = h
        stConv.pSrcData = self.buf
        stConv.nSrcDataLen = self.frame.nFrameLen
        stConv.enSrcPixelType = self.frame.enPixelType
        stConv.enDstPixelType = self._BGR8
        stConv.pDstBuffer = self.conv
        stConv.nDstBufferSize = len(self.conv)
        if self.cam.MV_CC_ConvertPixelType(stConv) != 0:
            return None
        img = np.frombuffer(self.conv, dtype=np.uint8, count=w * h * 3).reshape(h, w, 3)
        return img

    def release(self):
        try:
            self.cam.MV_CC_StopGrabbing()
            self.cam.MV_CC_CloseDevice()
            self.cam.MV_CC_DestroyHandle()
        except Exception:
            pass


class UvcCamera:
    def __init__(self, index=0):
        self.cap = cv2.VideoCapture(index)
        if not self.cap.isOpened():
            raise RuntimeError("无法打开 UVC 摄像头 %d" % index)

    def read(self):
        ok, f = self.cap.read()
        return f if ok else None

    def release(self):
        self.cap.release()


# ======================= 主循环 =======================
def send_vision(ser, x, c):
    try:
        ser.write(("C,%d,%d\n" % (x, c)).encode())
        return True
    except Exception:
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEF_PORT)
    ap.add_argument("--baud", type=int, default=DEF_BAUD)
    ap.add_argument("--uvc", type=int, default=None, help="强制用 UVC 摄像头编号")
    ap.add_argument("--cam", type=int, default=0, help="海康相机序号")
    ap.add_argument("--hz", type=int, default=DEF_SEND_HZ)
    ap.add_argument("--center", type=int, default=DEF_CENTER)
    ap.add_argument("--test", action="store_true", help="无相机, 发模拟摆动坐标")
    ap.add_argument("--no-show", action="store_true")
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=0.05)
    print("[SER] %s @ %d 已打开" % (args.port, args.baud))
    period = 1.0 / max(1, args.hz)
    last_send = 0.0
    last_log = 0.0
    center = args.center
    fail = 0
    sends = 0
    smooth = None

    if args.test:
        print("[TEST] 无相机, 发送模拟摆动坐标 (Ctrl-C 退出)")
        w = 640
        while True:
            x = int(w / 2 + (w / 2 - 20) * np.sin(time.time() * 2.0))
            c = w // 2
            ok = send_vision(ser, x, c)
            print("send x=%d c=%d %s" % (x, c, "OK" if ok else "FAIL"))
            time.sleep(0.1)

    # 打开相机
    cam = None
    if args.uvc is not None:
        cam = UvcCamera(args.uvc)
        print("[UVC] 摄像头", args.uvc)
    else:
        try:
            cam = HikCamera(args.cam)
        except Exception as e:
            print("[HIK] 失败:", e, "-> 回退 UVC 0")
            cam = UvcCamera(0)

    while True:
        bgr = cam.read()
        if bgr is None:
            continue
        if center < 0:
            center = bgr.shape[1] // 2
        cx, cy, mask = detect_green(bgr)

        now = time.time()
        # 时序平滑(EMA): 抑制 x 抖动
        if cx is not None:
            smooth = cx if smooth is None else (0.5 * smooth + 0.5 * cx)
            x_send = int(round(smooth))
        else:
            smooth = None
            x_send = None
        if x_send is not None and now - last_send >= period:
            last_send = now
            if send_vision(ser, x_send, center):
                fail = 0
                sends += 1
            else:
                fail += 1
                if fail % 30 == 0:
                    print("!! 串口发送失败, 检查端口 %s 是否被占用" % args.port)

        # 每秒状态
        if now - last_log >= 1.0:
            last_log = now
            gpx = int(np.count_nonzero(mask)) if mask is not None else 0
            print("[状态] 识别=%s x=%s err=%s 绿像素=%d 已发=%d %s" %
                  ("是" if cx is not None else "否", cx,
                   (cx - center) if cx is not None else "-", gpx, sends,
                   "" if sends else "<- 没发过: 检查绿光是否在画面内/阈值"))

        # 顺带回读 C 板遥测 (实时调试, 可忽略)
        try:
            while ser.in_waiting:
                tl = ser.readline().decode(errors="ignore").strip()
                if tl.startswith("D "):
                    print("[C]", tl)
        except Exception:
            pass

        if not args.no_show:
            disp = bgr.copy()
            cv2.line(disp, (center, 0), (center, disp.shape[0]), (255, 255, 0), 1)
            if cx is not None:
                cv2.circle(disp, (cx, cy), 6, (0, 0, 255), 2)
                cv2.putText(disp, "x=%d err=%d" % (cx, cx - center), (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            else:
                cv2.putText(disp, "no target", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            cv2.imshow("dart aim (q to quit)", disp)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
