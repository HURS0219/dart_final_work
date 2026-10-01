"""read_motor.py - 读 s_rt[slot] 与电机实例, 打印关键量(供方向/整定调试)

用法: python read_motor.py [slot]
"""
import re
import struct
import subprocess
import sys
import tempfile
import os

ELF = r"D:\worka\dart_final_work\make_one\build_dart_launcher_final\control-2026.elf"
NM = r"D:\workp\tea\pack\Toolchain\arm_gnu_toolchain\bin\arm-none-eabi-nm.exe"
JLINK = r"C:\Program Files\SEGGER\JLink_V966\JLink.exe"


def sym(name):
    out = subprocess.run([NM, "-S", ELF], capture_output=True, text=True).stdout
    for line in out.splitlines():
        t = line.split()
        if len(t) >= 3 and t[-1] == name:
            return int(t[0], 16)
    return None


def rd(addr, ln):
    """读内存。**分段读**(每段 <=64 字节): J-Link 一次 dump 太长会丢行/截断。
    输出形如: 20000770 = 00 00 A0 41 ...  ...A............
    行首地址, 每行 16 字节, 行尾 ASCII 注解需丢弃。"""
    data = bytearray()
    off = 0
    while off < ln:
        chunk = min(64, ln - off)
        a = addr + off
        scr = "si SWD\nspeed 4000\ndevice STM32F407IG\nconnect\nh\nmem 0x%08X, %d\nqc\n" % (a, chunk)
        f = os.path.join(tempfile.gettempdir(), "rm.jlink")
        open(f, "w").write(scr)
        out = subprocess.run([JLINK, "-NoGui", "1", "-CommanderScript", f],
                             capture_output=True, text=True).stdout
        got = bytearray()
        for line in out.splitlines():
            m = re.match(r"^([0-9A-Fa-f]{8})\s*=\s*((?:[0-9A-Fa-f]{2}\s+)*[0-9A-Fa-f]{2})",
                         line.strip())
            if not m:
                continue
            base = int(m.group(1), 16)
            vals = [int(x, 16) for x in m.group(2).split()]
            if base == a:
                got = bytearray(vals)
            elif got and base == a + len(got):
                got += bytes(vals)
        if not got:
            break
        data += got
        off += len(got)
    return bytes(data)


slot = int(sys.argv[1]) if len(sys.argv) > 1 else 0
srt = sym("s_rt")
if srt is None:
    print("找不到 s_rt")
    sys.exit(1)

rt = rd(srt + slot * 0x30, 0x30)
inst = struct.unpack_from("<I", rt, 0)[0]
zero = struct.unpack_from("<f", rt, 4)[0]
zero_valid = rt[8]
target = struct.unpack_from("<f", rt, 28)[0]
mode = rt[24]

print("=== s_rt[%d] @0x%08X ===" % (slot, srt + slot * 0x30))
print("  inst       = 0x%08X" % inst)
print("  zero       = %.4f" % zero)
print("  zero_valid = %d" % zero_valid)
print("  mode       = %d (0=STOP 2=ANGLE 1=SPEED)" % mode)
print("  target     = %.4f" % target)

if inst == 0:
    print("  (inst 为空)")
    sys.exit(0)

# ---- 偏移必须与编译期一致! 由 app/offset_probe.c 实测得出(勿手改):
#   sizeof(DJIMotorInstance)=440, sizeof(Measure)=24, sizeof(Settings)=7
#   measure@0  settings@24  controller@32
#   controller 内: angle_PID@224(相对32 -> 绝对256)  pid_ref@376  final_output@380
#   sender_group@420  message_num@421  motor_type@422  stop_flag@423
#   feed_cnt@428
OFF_PID_REF = 408   # 由 DJIMotorSetPIDRef 反汇编确认(vstr s0,[r0,#408])
OFF_FINAL_OUT = 412 # 反汇编确认(vstr s0,[r0,#412])
OFF_ANGLE_PID = 288 # 由 DJIMotorInit 反汇编确认(add.w r0, r4, #288 -> 第3次 PIDInit)
OFF_TOTAL_ANGLE = 16
OFF_SPEED_APS = 8
OFF_REAL_CUR = 12
OFF_TEMP = 14
OFF_SETTINGS = 24
OFF_SENDER_GROUP = 420
OFF_MESSAGE_NUM = 421
OFF_STOP_FLAG = 423
OFF_FEED_CNT = 428

m = rd(inst, 0x1C0)
total = struct.unpack_from("<f", m, OFF_TOTAL_ANGLE)[0]
spd = struct.unpack_from("<f", m, OFF_SPEED_APS)[0]
cur = struct.unpack_from("<h", m, OFF_REAL_CUR)[0]
temp = m[OFF_TEMP]
settings = list(m[OFF_SETTINGS:OFF_SETTINGS + 7])
pid_ref = struct.unpack_from("<f", m, OFF_PID_REF)[0]
fout = struct.unpack_from("<f", m, OFF_FINAL_OUT)[0]
stop_flag = m[OFF_STOP_FLAG]
feed = struct.unpack_from("<I", m, OFF_FEED_CNT)[0]
sgrp = m[OFF_SENDER_GROUP]
mnum = m[OFF_MESSAGE_NUM]
# angle_PID 头部: Kp Ki Kd MaxOut IntegralLimit DeadBand Improve ... (顺序见 controller.h)
kp = struct.unpack_from("<f", m, OFF_ANGLE_PID + 0)[0]
ki = struct.unpack_from("<f", m, OFF_ANGLE_PID + 4)[0]
kd = struct.unpack_from("<f", m, OFF_ANGLE_PID + 8)[0]

print("=== 电机实例 0x%08X ===" % inst)
print("  total_angle  = %.4f" % total)
print("  speed_aps    = %.1f deg/s" % spd)
print("  real_current = %d" % cur)
print("  temperature  = %d C" % temp)
print("  settings     = %s  (outer, close, motor_rev, fb_rev, ...)" % settings)
print("  stop_flag    = %d (0=STOP 1=ENABLED)" % stop_flag)
print("  feed_cnt     = %d" % feed)
print("  sender_group = %d   message_num = %d" % (sgrp, mnum))
print("  pid_ref      = %.2f" % pid_ref)
print("  final_output = %.2f" % fout)
print("  angle_PID    = Kp=%.3f Ki=%.3f Kd=%.3f" % (kp, ki, kd))
print()
# 方向符号: 取 motor_reverse_flag(motortest 的 SignOf 同此)
rev = settings[2]  # motor_settings 内 motor_reverse_flag(第 3 个字节)
sgn = -1.0 if rev == 1 else 1.0
outang = sgn * (total - zero) / 19.2032
print("=== 换算(ratio=19.2032) ===")
print("  输出侧角度 = SignOf*(%.3f - %.3f)/19.2032 = %.3f deg  (SignOf=%.0f)" % (total, zero, outang, sgn))
print("  与 target 差 = %.3f deg" % (target - outang))
