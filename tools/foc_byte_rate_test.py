# -*- coding: utf-8 -*-
"""区分"遥测停发"与"数据冻结": disable 后统计原始字节流"""
import struct
import time

import serial

ser = serial.Serial("COM44", 6500000, timeout=0.02, write_timeout=0.5)


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


cmd = lambda c, w=0.4: (ser.reset_input_buffer(),
                        ser.write((c + "\n").encode()),
                        time.sleep(w), ser.read(65536)) and None

ser.reset_input_buffer()
ser.write(b"log 1\n")
time.sleep(0.5)
ser.read(65536)

ser.write(b"mode vf\n"); time.sleep(0.3)
ser.write(b"enable\n"); time.sleep(0.4)
ser.write(b"vq 0.25\n"); time.sleep(0.2)
ser.write(b"rpm 200\n"); time.sleep(3)

# disable 前先记录基线字节率
buf_pre = bytearray()
t0 = time.time()
while time.time() - t0 < 1.0:
    ch = ser.read(16384)
    if ch:
        buf_pre.extend(ch)
print("RUN 期 1s 字节: %d (~%.0f 帧)" % (len(buf_pre), len(buf_pre) / 68.0))

ser.write(b"disable\n")
time.sleep(0.1)

# disable 后分段统计字节量
for k in range(6):
    buf = bytearray()
    t0 = time.time()
    while time.time() - t0 < 1.0:
        ch = ser.read(16384)
        if ch:
            buf.extend(ch)
    fr = parse_frames(bytes(buf))
    # theta_e = ch0
    ths = [f[0] for f in fr]
    th_span = (max(ths) - min(ths)) if ths else 0.0
    print("coast+%ds: %5d B (~%4d 帧) theta span=%.3f" % (
        k + 1, len(buf), len(fr), th_span))

ser.write(b"log 0\n")
time.sleep(0.3)
