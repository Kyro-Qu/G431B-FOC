# -*- coding: utf-8 -*-
"""恒流 iq=1.5A + offset 全相位扫描 (32 步, 11.25度步进)
判据: 每步跑 1.5s, 若 1.5s 内 pos 位移 > 4 rad (持续转动) 则该 offset 有效
输出: 位移 vs offset 极坐标分布 -> 正确 offset 是唯一位移峰
"""
import math
import re
import time

import serial

ser = serial.Serial("COM44", 6500000, timeout=0.05, write_timeout=0.5)


def cmd(c, wait=0.4):
    ser.reset_input_buffer()
    ser.write((c + "\n").encode())
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        ch = ser.read(4096)
        if ch:
            buf.extend(ch)
    return buf.decode("ascii", "replace")


def get_pos():
    s = cmd("status", 0.3)
    m = re.search(r"pos=(-?[\d.]+)rad", s)
    return float(m.group(1)) if m else 0.0


cmd("fault clear", 0.4)
cmd("mode iq", 0.3)
print("offset_deg | pos_delta_rad | status")
results = []
for i in range(32):
    off = i * 0.19635  # 11.25 deg
    cmd("calib offset %.4f" % off, 0.25)
    cmd("enable", 0.4)
    p0 = get_pos()
    cmd("target 1.5", 0.2)
    time.sleep(1.5)
    p1 = get_pos()
    d = p1 - p0
    s = cmd("status", 0.3)
    sm = re.search(r"M0 (\w+)", s)
    st = sm.group(1) if sm else "?"
    cmd("disable", 0.3)
    cmd("fault clear", 0.3)
    results.append((i * 11.25, d, st))
    flag = " <<<< SPIN" if abs(d) > 4.0 else ""
    print("%9.2f | %+7.2f | %s%s" % (i * 11.25, d, st, flag))
    time.sleep(0.4)
# 汇总: 最大位移的 offset
best = max(results, key=lambda r: abs(r[1]))
print("\n最大位移: offset=%.2f deg, d=%.2f rad, %s" % best)
spins = [r for r in results if abs(r[1]) > 4.0]
print("连续转动区间: %s" % [(round(r[0], 1), round(r[1], 1)) for r in spins])
