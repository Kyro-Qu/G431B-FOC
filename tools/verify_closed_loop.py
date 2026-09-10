# -*- coding: utf-8 -*-
"""电机参数自动辨识后闭环运行验证 (COM44)"""
import sys
import time
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def send(ser, cmd):
    ser.write((cmd + "\n").encode("ascii"))
    time.sleep(0.08)
    if ser.in_waiting:
        return ser.read(ser.in_waiting).decode("ascii", errors="replace")
    return ""

def main():
    print("==================================================")
    print(">>> 验证自动辨识参数应用后的速度闭环响应性能 <<<")
    print("==================================================")
    ser = serial.Serial(PORT, BAUD, timeout=0.1, write_timeout=0.5)
    time.sleep(0.2)
    ser.reset_input_buffer()

    send(ser, "log 0")
    send(ser, "fault clear")

    # 1. 检查当前参数
    print("\n[1/4] 查看应用后的电机参数 (conf read):")
    res = send(ser, "conf read")
    for line in res.splitlines():
        if any(k in line for k in ("pp", "rs", "ls", "ke", "cur_bw")):
            print(" ", line)

    # 2. 切换至速度模式
    print("\n[2/4] 切入速度模式 (mode vel, target 600 RPM):")
    send(ser, "mode vel")
    send(ser, "target 600")
    send(ser, "enable")
    time.sleep(1.0)

    # 3. 采样闭环稳态转速与 Iq 电流
    print("\n[3/4] 稳态转速跟踪采样 (目标 600 RPM):")
    for _ in range(5):
        st = send(ser, "status")
        for line in st.splitlines():
            if any(k in line for k in ("state=", "vel=", "iq=", "id=", "vbus=")):
                print(" ", line)
        time.sleep(0.3)

    # 4. 停机并清除故障
    print("\n[4/4] 闭环运行结束，平稳停机 (disable):")
    send(ser, "disable")
    send(ser, "log 1")
    ser.close()
    print("测试圆满完成。")

if __name__ == "__main__":
    main()
