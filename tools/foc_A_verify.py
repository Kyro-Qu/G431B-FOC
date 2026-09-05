# -*- coding: utf-8 -*-
"""A 验证 v4: 新固件(0.25V calib) + conf erase 默认参数(Ls=20uH Kp=0.040)
calib full -> mode vel -> enable -> 2000rpm 25s 稳定性测试。
"""
import re
import serial
import time
import sys

PORT = "COM44"
BAUD = 6500000
DUR_S = 25.0
TARGET_RPM = 2000.0

ser = serial.Serial(PORT, BAUD, timeout=0.05, write_timeout=0.5)
buf = bytearray()


def clear_buf():
    buf[:] = b""


def read_some():
    try:
        chunk = ser.read(4096)
        if chunk:
            buf.extend(chunk)
    except serial.SerialException:
        pass


def cmd(c, wait=0.35):
    clear_buf()
    ser.reset_input_buffer()
    try:
        ser.write((c + "\n").encode("ascii"))
    except Exception as e:
        print("TX fail: %r" % e)
        return ""
    t0 = time.time()
    while time.time() - t0 < wait:
        read_some()
    return buf.decode("ascii", "replace")


def cmd_tail(c, wait=0.35, n=4):
    lines = re.findall(r"[\x20-\x7e]{6,}", cmd(c, wait))
    return " | ".join(lines[-n:])


def get_state():
    r = cmd("status", 0.3)
    mm = re.search(r"M0 (\w+) mode=(\w+)", r)
    if mm:
        return mm.group(1), mm.group(2)
    return "?", "?"


def fault_code():
    r = cmd_tail("fault", 0.3, 1)
    mm = re.search(r"fault=(\d+)", r)
    return mm.group(1) if mm else "?"


print("=== A v4: 0.25V calib + default 20uH -> 2000rpm 25s ===")

st, md = get_state()
print("1) boot: %s/%s" % (st, md))
if st not in ("IDLE",):
    cmd("disable", 0.5)
    cmd("fault clear", 0.5)
    st, md = get_state()
    print("   after clear: %s/%s" % (st, md))

print("2) conf state check (skip erase: flash already erased in v2):")
print("   kp check:", cmd_tail("current bw 2000", 0.5, 1))

print("3) calib full ...")
r = cmd_tail("calib full", 1.0, 1)
print("   start:", r)
t_calib = time.time()
st, md = "?", "?"
for i in range(50):
    time.sleep(0.5)
    st, md = get_state()
    if st == "IDLE":
        print("   calib done after %.1fs" % (time.time() - t_calib))
        break
    if st == "FAULT":
        print("   calib FAULT at %.1fs, code=%s" %
              (time.time() - t_calib, fault_code()))
        sys.exit(1)
else:
    print("   WARN: calib not done, state=%s" % st)

print("4) calib offset:", cmd_tail("calib offset", 0.3, 1))
print("5) mode vel:", cmd_tail("mode vel", 0.4, 1))
print("6) enable:", cmd_tail("enable", 0.5, 1))
print("7) target %.0f, run %.0fs" % (TARGET_RPM, DUR_S))
cmd("target %.0f" % TARGET_RPM, 0.4)

t0 = time.time()
last_poll = t0
states_seen = []
while time.time() - t0 < DUR_S:
    read_some()
    if time.time() - last_poll >= 2.0:
        st, md = get_state()
        states_seen.append("%5.1fs %s/%s" % (time.time() - t0, st, md))
        last_poll = time.time()
        if st == "FAULT":
            print("   FAULT at %.1fs, code=%s" %
                  (time.time() - t0, fault_code()))
            break
for s in states_seen:
    print("  ", s)

print("8) final status:", cmd("status", 0.6).replace("\r\n", " ~ ")[:300])
print("9) disable:", cmd_tail("disable", 0.5, 1))
st, md = get_state()
print("10) post: %s/%s" % (st, md))

if st == "?":
    print("RESULT: FAILED - link lost (BOR?)")
elif "FAULT" in states_seen[-1]:
    print("RESULT: FAILED - FAULT during run")
elif st == "RUN":
    print("RESULT: PASSED - 2000rpm 25s stable with default 20uH")
else:
    print("RESULT: UNKNOWN - state=%s" % st)
ser.close()
