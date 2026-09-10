# -*- coding: utf-8 -*-
"""电机参数辨识实机测试脚本 (COM44)"""
import sys
import time
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def send_line(ser, line):
    ser.write((line + "\n").encode("ascii"))
    time.sleep(0.05)

def wait_and_print(ser, max_wait=8.0, stop_phrases=("ident DONE", "ident FAIL")):
    t0 = time.time()
    collected = []
    while time.time() - t0 < max_wait:
        n = ser.in_waiting
        if n:
            chunk = ser.read(n).decode("ascii", errors="replace")
            sys.stdout.write(chunk)
            sys.stdout.flush()
            collected.append(chunk)
            text = "".join(collected)
            for sp in stop_phrases:
                if sp in text:
                    time.sleep(0.1)
                    if ser.in_waiting:
                        chunk2 = ser.read(ser.in_waiting).decode("ascii", errors="replace")
                        sys.stdout.write(chunk2)
                        sys.stdout.flush()
                    return "".join(collected)
        time.sleep(0.02)
    return "".join(collected)

def main():
    print("连接 COM44 @ %d..." % BAUD)
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 关遥测
    send_line(ser, "log 0")
    time.sleep(0.1)
    ser.reset_input_buffer()

    print("\n----------------- 1. 单项测试: 极对数辨识 (ident pp) -----------------")
    send_line(ser, "ident pp")
    wait_and_print(ser, max_wait=8.0)

    print("\n----------------- 2. 查看辨识结果缓存 (ident show) -----------------")
    send_line(ser, "ident show")
    time.sleep(0.2)
    if ser.in_waiting:
        print(ser.read(ser.in_waiting).decode("ascii", errors="replace"))

    print("\n----------------- 3. 单项测试: 磁链与 Ke 辨识 (ident flux) -----------------")
    send_line(ser, "ident flux")
    wait_and_print(ser, max_wait=8.0)

    print("\n----------------- 4. 查看辨识结果缓存 (ident show) -----------------")
    send_line(ser, "ident show")
    time.sleep(0.2)
    if ser.in_waiting:
        print(ser.read(ser.in_waiting).decode("ascii", errors="replace"))

    # 恢复遥测
    send_line(ser, "log 1")
    ser.close()
    print("\n测试完成。")

if __name__ == "__main__":
    main()
