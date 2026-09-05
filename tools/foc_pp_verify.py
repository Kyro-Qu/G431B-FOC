# -*- coding: utf-8 -*-
"""手转一圈电圈数验证: pp 与 CPR 正确性
iq 模式 0A enable (角度源活, 无转矩), 手转一圈, 数 theta_e 净旋转
"""
import re
import serial
import struct
import time

ser = serial.Serial("COM44", 6500000, timeout=0.02, write_timeout=0.5)


def cmd(c, wait=0.4):
    ser.reset_input_buffer()
    ser.write((c + "\n").encode())
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        ch = ser.read(8192)
        if ch:
            buf.extend(ch)
    return buf.decode("ascii", "replace")


def parse_frames(data):
    tail = bytes([0, 0, 0x80, 0x7F])
    frames = []
    i = 0
    while True:
        j = data.find(tail, i)
        if j < 0 or j < 64:
            break
        frames.append(struct.unpack("<16f", data[j - 64:j]))
        i = j + 4
    return frames


cmd("fault clear", 0.4)
cmd("mode iq", 0.3)
cmd("enable", 0.4)
cmd("target 0", 0.3)
cmd("log 1", 0.5)
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 14.0:
    ch = ser.read(16384)
    if ch:
        buf.extend(ch)
frames = parse_frames(bytes(buf))
cmd("disable", 0.4)
cmd("log 0", 0.3)
print("frames:", len(frames))
if not frames:
    raise SystemExit(1)
th = [f[0] for f in frames]
total = 0.0
prev = th[0]
for v in th[1:]:
    d = v - prev
    if d > 3.14159:
        d -= 6.28319
    if d < -3.14159:
        d += 6.28319
    total += d
    prev = v
print("theta_e net rotation: %.2f rad = %.2f elec rev (hand 1 mech rev should be 6.00)" % (
    total, total / 6.28319))
