# -*- coding: utf-8 -*-
"""位置模式阻尼整定验证: 阶跃响应 + 扰动恢复监测 (COM44)"""
import re
import time
import serial

PORT = "COM44"
BAUD = 6500000

def send(ser, cmd, delay=0.1, retries=3):
    for _ in range(retries):
        ser.reset_input_buffer()
        ser.write((cmd + "\n").encode("ascii"))
        time.sleep(delay)
        t0 = time.time()
        buf = bytearray()
        while time.time() - t0 < delay + 0.15:
            n = ser.in_waiting
            if n:
                buf.extend(ser.read(n))
            time.sleep(0.01)
        out = buf.decode("ascii", "replace").strip()
        if out:
            return out
    return ""

def grab(st, key):
    m = re.search(r"%s(-?[0-9.]+)" % re.escape(key), st)
    return float(m.group(1)) if m else None

def sample(ser, seconds, period=0.1):
    rows = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        st = send(ser, "status", delay=0.05)
        rows.append((time.time() - t0,
                     grab(st, "pos"), grab(st, "vel="), grab(st, "iq=")))
        time.sleep(period)
    return rows

def show(rows, title):
    print("\n--- %s ---" % title)
    for t, pos, vel, iq in rows:
        if pos is not None:
            print("  t=%5.2fs pos=%8.3frad vel=%7.1frpm iq=%6.2fA"
                  % (t, pos, vel or 0, iq or 0))

def main():
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    time.sleep(0.2)
    try:
        send(ser, "log 0")
        send(ser, "disable")
        send(ser, "fault clear")

        st = ""
        for _ in range(5):
            st = send(ser, "status")
            if "calib=" in st:
                break
            time.sleep(0.2)
        if grab(st, "calib=") != 1.0:
            print("未校准, 执行 calib...")
            send(ser, "calib")
            for _ in range(40):
                time.sleep(0.5)
                st = send(ser, "status")
                cal = grab(st, "calib=")
                if "M0 IDLE" in st or "M0 RUN" in st:
                    break
            print("calib done:", grab(st, "calib="))
            if grab(st, "calib=") != 1.0:
                print("校准未成功, 退出")
                return

        print("\n[1] mode pos + enable")
        print(" ->", send(ser, "mode pos"))
        print(" ->", send(ser, "enable"))
        time.sleep(0.3)

        origin = grab(send(ser, "status"), "pos=")
        print(" origin=%.3frad" % origin)

        print("\n[2] 静止锁定观察 1s (阻尼开启后应无抖动)")
        show(sample(ser, 1.0), "hold origin")

        print("\n[3] target 3.14 阶跃")
        print(" ->", send(ser, "target 3.14"))
        show(sample(ser, 2.0), "step +3.14")

        print("\n[4] target 0 回原点")
        print(" ->", send(ser, "target 0"))
        show(sample(ser, 2.0), "step back 0")

        print("\n[5] 手拨扰动窗口: 请现在用手拨动电机, 大角度, 5 秒窗口")
        show(sample(ser, 5.0, period=0.08), "disturbance")

        print("\n[6] 停机")
        send(ser, "disable")
    finally:
        try:
            send(ser, "disable")
            ser.close()
        except Exception:
            pass

if __name__ == "__main__":
    main()
