#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mem_read.py — 用 arm-none-eabi-nm 解析符号 + J-Link 读目标 RAM/Flash, 并按类型解码。

用途: 不接调试 UI, 也能结构化地读固件里的变量/结构体(排查状态/参数很方便)。

用法:
  # 读符号 g_servo 处的 21 个 word; 把下标 7,8,9,... 按 float 打印
  python mem_read.py --elf make_one/build_servo_test/control-2026.elf \
      --sym g_servo --words 21 --floats 7 8 9 10 11 12 13 14 15 17 18 19

  # g_servo 存的是指针: 先读指针再加 --deref
  python mem_read.py --elf <elf> --sym g_servo --deref --words 21

  # 直接读地址(如 Flash bank):
  python mem_read.py --elf <elf> --addr 0x080C0000 --words 3

说明: 每次重编符号地址会变, 本脚本每次动态解析, 无需改地址。
"""
import argparse
import os
import re
import struct
import subprocess
import sys
import tempfile

DEFAULT_NM = r"D:\workp\tea\pack\Toolchain\arm_gnu_toolchain\bin\arm-none-eabi-nm.exe"
DEFAULT_JLINK = r"C:\Program Files\SEGGER\JLink_V966\JLink.exe"


def elf_symbol(nm, elf, name):
    """在 ELF 符号表里查 name 的地址; 找不到返回 None。"""
    out = subprocess.run([nm, elf], capture_output=True, text=True).stdout
    for line in out.splitlines():
        tok = line.split()
        if tok and tok[-1] == name:
            return int(tok[0], 16)
    return None


def jlink_read_words(jlink, device, addr, n):
    """用 J-Link 读取 addr 起始的 n 个 32-bit word。"""
    script = ("si SWD\nspeed 4000\ndevice %s\nconnect\nh\nmem32 0x%X %d\ng\nqc\n"
              % (device, addr, n))
    tmp = os.path.join(tempfile.gettempdir(), "tmp_memread.jlink")
    with open(tmp, "w") as f:
        f.write(script)
    out = subprocess.run([jlink, "-NoGui", "1", "-CommanderScript", tmp],
                         capture_output=True, text=True).stdout
    words = []
    for line in out.splitlines():
        m = re.match(r"\s*([0-9A-Fa-f]{8}) = (.*)", line)
        if m:
            for w in m.group(2).split():
                words.append(int(w, 16))
    return words


def as_float(w):
    return struct.unpack("<f", struct.pack("<I", w))[0]


def main():
    ap = argparse.ArgumentParser(description="nm + J-Link 读内存并解码")
    ap.add_argument("--elf", required=True, help="目标固件 ELF")
    ap.add_argument("--sym", help="符号名(把符号地址作为起始地址)")
    ap.add_argument("--addr", help="直接地址(如 0x080C0000)")
    ap.add_argument("--deref", action="store_true", help="把符号/地址处的 1 个 word 当作指针, 再读其内容")
    ap.add_argument("--words", type=int, default=1, help="读取的 word 数(默认 1)")
    ap.add_argument("--floats", type=int, nargs="*", default=[],
                    help="按 float 打印的 word 下标列表")
    ap.add_argument("--nm", default=DEFAULT_NM)
    ap.add_argument("--jlink", default=DEFAULT_JLINK)
    ap.add_argument("--device", default="STM32F407IG")
    args = ap.parse_args()

    if args.addr is not None:
        addr = int(args.addr, 0)
    elif args.sym is not None:
        addr = elf_symbol(args.nm, args.elf, args.sym)
        if addr is None:
            print("符号未找到:", args.sym)
            sys.exit(1)
    else:
        print("需要 --sym 或 --addr")
        sys.exit(1)

    if args.deref:
        addr = jlink_read_words(args.jlink, args.device, addr, 1)[0]

    words = jlink_read_words(args.jlink, args.device, addr, args.words)
    print("addr=0x%08X  words=%d" % (addr, len(words)))
    print("hex : " + " ".join("%08X" % w for w in words))
    for i in args.floats:
        if 0 <= i < len(words):
            print("  [%2d] float = %g   (0x%08X)" % (i, as_float(words[i]), words[i]))


if __name__ == "__main__":
    main()
