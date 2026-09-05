# -*- coding: utf-8 -*-
"""低负载滑行编码器测试: vq 0.25V rpm 200 (电流~1A 内, 不超电源限流)
若低负载下滑行不冻结 -> 1.5A 电源塌陷是编码器死亡根因
"""
import struct
import time

import serial

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
cmd("log 1", 0.4)
cmd("mode vf", 0.3)
cmd("enable", 0.3)
cmd("vq 0.25", 0.2)
cmd("rpm 200", 0.2)
time.sleep(3)
ser.write(b"disable\n")
time.sleep(0.1)
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 5.0:
    ch = ser.read(16384)
    if ch:
        buf.extend(ch)
frames = parse_frames(bytes(buf))
print("captured %d frames over 5s coast" % len(frames))
import statistics
seg_n = len(frames) // 10
for k in range(10):
    seg = frames[k * seg_n:(k + 1) * seg_n]
    if not seg:
        continue
    th = [f[0] for f in seg]
    vr = [f[2] for f in seg]
    print("  seg%d (%.1fs): theta span=%.3f vel mean=%+7.1f" % (
        k, k * seg_n / 500.0, max(th) - min(th), statistics.mean(vr)))
cmd("log 0", 0.3)
