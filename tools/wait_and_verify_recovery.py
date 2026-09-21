# -*- coding: utf-8 -*-
"""
等待用户断电/供电/重新拔插 USB 后，自动检测并验证 COM44 与 SWD 恢复情况
"""
import time
import serial
import serial.tools.list_ports

PORT = "COM44"
BAUD = 6500000

def wait_for_port():
    print(">>> 正在等待 COM44 重新枚举并就绪...")
    timeout = 30.0
    t0 = time.time()
    while time.time() - t0 < timeout:
        ports = [p.device for p in serial.tools.list_ports.comports()]
        if PORT in ports:
            print(f"  [发现] {PORT} 已在系统中枚举！")
            time.sleep(1.0) # 等待驱动就绪
            return True
        time.sleep(0.5)
    print(f"  [超时] {timeout} 秒内未检测到 {PORT}。")
    return False

def verify_communication():
    print(f"\n>>> 正在验证 {PORT} @ {BAUD} 双向通信...")
    for i in range(5):
        try:
            ser = serial.Serial(PORT, BAUD, timeout=0.2)
            time.sleep(0.1)
            ser.reset_input_buffer()
            ser.write(b"version\r\n")
            t0 = time.time()
            data = bytearray()
            while time.time() - t0 < 0.4:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk: data.extend(chunk)
            ser.close()
            if len(data) > 0:
                print(f"  [成功] 第 {i+1} 次测试收到 {len(data)} 字节回复:")
                print(f"  --> {repr(bytes(data[:120]))}")
                return True
            else:
                print(f"  [重试] 第 {i+1} 次测试收到 0 字节，正在重试...")
        except Exception as e:
            print(f"  [重试] 第 {i+1} 次测试串口打开失败: {e}")
        time.sleep(0.5)
    return False

if __name__ == "__main__":
    if wait_for_port():
        if verify_communication():
            print("\n==================================================")
            print("  >>> COM44 串口已完全恢复双向正常通信！ <<<       ")
            print("==================================================")
        else:
            print("\n==================================================")
            print("  >>> COM44 仍未收到有效回复，请检查接线或重新执行 <<<")
            print("==================================================")
