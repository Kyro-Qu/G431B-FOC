# -*- coding: utf-8 -*-
"""全套电机参数自动辨识 (Full Suite) 端到端测试脚本 (COM44)"""
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

def wait_and_print(ser, max_wait=15.0, stop_phrases=("ident DONE", "ident FAIL")):
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
    print("==================================================")
    print(">>> 连接 COM44 @ %d，开始全套参数自动辨识测试 <<<" % BAUD)
    print("==================================================")
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # 1. 关遥测
    send_line(ser, "log 0")
    time.sleep(0.1)
    ser.reset_input_buffer()

    # 2. 发送全套辨识指令
    print("\n[1/3] 启动全套辨识: ident full ...")
    send_line(ser, "ident full")
    res = wait_and_print(ser, max_wait=15.0)

    # 3. 查看辨识结果汇总
    print("\n[2/3] 查看辨识结果汇总 (ident show):")
    send_line(ser, "ident show")
    time.sleep(0.2)
    if ser.in_waiting:
        print(ser.read(ser.in_waiting).decode("ascii", errors="replace"))

    # 4. 测试参数应用
    print("\n[3/3] 测试应用参数 (ident apply):")
    send_line(ser, "ident apply")
    time.sleep(0.2)
    if ser.in_waiting:
        print(ser.read(ser.in_waiting).decode("ascii", errors="replace"))

    # 5. 恢复遥测
    send_line(ser, "log 1")
    ser.close()
    print("\n全套辨识端到端测试完成。")

if __name__ == "__main__":
    main()
