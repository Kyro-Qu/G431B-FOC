# -*- coding: utf-8 -*-
"""滑行中编码器 theta_e 跟随测试:
vf 拖到 400rpm -> disable(不切闭环) -> 滑行期抓遥测
ch0=theta_e ch2=vel_rpm ch15=vel_filt
若滑行中 theta_e 连续变化/vel_rpm 有读数 -> 编码器链路活, 问题在闭环角度分支
若滑行中全冻结 -> 编码器读数本身在 PWM 工作后失效
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
cmd("log 1", 0.4)
cmd("mode vf", 0.3)
cmd("enable", 0.3)
cmd("vq 0.6", 0.2)
cmd("rpm 400", 0.2)
time.sleep(3)
# 只 disable, 惯性滑行, 保持 vf 模式
t_sw = time.time()
ser.write(b"disable\n")
time.sleep(0.1)
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 4.0:
    ch = ser.read(16384)
    if ch:
        buf.extend(ch)
frames = parse_frames(bytes(buf))
print("captured %d frames over 4s coast" % len(frames))
# 分段统计
seg_n = len(frames) // 8
for k in range(8):
    seg = frames[k * seg_n:(k + 1) * seg_n]
    if not seg:
        continue
    th = [f[0] for f in seg]
    vr = [f[2] for f in seg]
    import statistics
    print("  seg%d (%.1fs): theta std=%.3f span=%.3f vel mean=%+7.1f max=%7.1f" % (
        k, k * seg_n / 500.0, statistics.pstdev(th),
        max(th) - min(th), statistics.mean(vr), max(vr)))
cmd("log 0", 0.3)
