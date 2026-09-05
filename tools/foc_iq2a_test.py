# -*- coding: utf-8 -*-
"""力矩模式 2A 恒流测试: 分辨换相错误 vs 起步转矩饥饿"""
import re
import serial
import time

ser = serial.Serial("COM44", 6500000, timeout=0.05, write_timeout=0.5)


def cmd(c, wait=0.5):
    ser.reset_input_buffer()
    ser.write((c + "\n").encode())
    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait:
        ch = ser.read(4096)
        if ch:
            buf.extend(ch)
    return buf.decode("ascii", "replace")


def tail(c, wait=0.5, n=1):
    ls = re.findall(r"[\x20-\x7e]{6,}", cmd(c, wait))
    return " | ".join(ls[-n:])


cmd("fault clear", 0.4)
print("mode iq:", tail("mode iq", 0.4))
print("enable:", tail("enable", 0.5))
print("target 2.0A:", tail("target 2.0", 0.4))
pos0 = None
for i in range(6):
    time.sleep(1.0)
    s = cmd("status", 0.4)
    pm = re.search(r"pos=(-?[\d.]+)rad", s)
    vm = re.search(r"vel_obs=(-?[\d.]+)rpm", s)
    im = re.search(r"iq=(-?[\d.]+)A iq_ref=(-?[\d.]+)A", s)
    p = float(pm.group(1)) if pm else 0.0
    if pos0 is None:
        pos0 = p
    print("t+%d: pos=%.3f (d=%.3f) vel_obs=%s iq=%s" % (
        i + 1, p, p - pos0, vm.group(1) if vm else "?",
        im.group(1) if im else "?"))
print("disable:", tail("disable", 0.4))
