# -*- coding: utf-8 -*-
"""
tools/trace_case1.py
Case 1 专用调试与遥测跟踪脚本
先清除故障，确保 calib=1* 且 IDLE，再启动 800 RPM 稳态监控
"""
import sys
import time
import serial

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def send_cmd(ser, cmd, delay=0.05):
    ser.reset_input_buffer()
    ser.write((cmd + '\r\n').encode('ascii'))
    time.sleep(delay)
    buf = ""
    while ser.in_waiting:
        buf += ser.read(ser.in_waiting).decode('ascii', errors='replace')
        time.sleep(0.01)
    return buf.strip()

def main():
    print("=" * 80)
    print(">>> 启动 Case 1 稳态建立追踪测试 <<<")
    print("=" * 80)

    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print(f"[-] 打开串口 {PORT} 失败: {e}")
        return 1

    time.sleep(0.05)

    try:
        # 1. 彻底清除已有故障与停机状态
        print("\n[步骤 1] 清除故障与重置...")
        print("  -> fault clear:", send_cmd(ser, "fault clear"))
        print("  -> enc fault clear:", send_cmd(ser, "enc fault clear"))
        time.sleep(0.1)

        # 2. 检查 calib
        st = send_cmd(ser, "status")
        if "calib=1*" not in st:
            print("  执行 calib...")
            send_cmd(ser, "calib")
            for _ in range(40):
                time.sleep(0.3)
                st_w = send_cmd(ser, "status")
                if "calib=1*" in st_w and "M0 IDLE" in st_w:
                    print("  [+] calib=1* 就绪")
                    break
        else:
            print("  [+] calib=1* 已就绪")

        # 3. 配置并使能
        print("\n[步骤 2] 配置并使能 800 RPM...")
        cmds = [
            "feedback auto",
            "feedback speed 420 320",
            "mode vel",
            "vel ramp 800",
            "target 800",
            "enable"
        ]
        for c in cmds:
            r = send_cmd(ser, c)
            print(f"  -> {c:24s} => {r}")

        # 4. 持续采样 6 秒，观察 conf_win, conf_inst, streak, qual 的建立过程
        print("\n[步骤 3] 持续 6 秒遥测跟踪 (每 0.3s 查询)...")
        print(f"{'Time':^6}|{'Vel(RPM)':^9}|{'Lock':^6}|{'C_Inst':^8}|{'C_Win':^7}|{'Err(deg)':^9}|{'SpdErr':^8}|{'RMS(deg)':^9}|{'Peak':^7}|{'Streak':^8}|{'Qual':^8}")
        print("-" * 95)

        t0 = time.time()
        while time.time() - t0 < 6.0:
            t_curr = time.time() - t0
            st = send_cmd(ser, "status", 0.03)
            sl = send_cmd(ser, "sensorless status", 0.03)
            vel = 0.0
            lock = 0
            streak = 0
            qual = 0
            c_inst = 0.0
            c_win = 0.0
            err_deg = 0.0
            spd_err = 0.0
            rms = 0.0
            peak = 0.0

            for l in st.splitlines():
                if l.startswith("vel="):
                    try: vel = float(l.split("=")[1].replace("rpm",""))
                    except: pass
            for l in sl.splitlines():
                if l.startswith("sensorless:"):
                    import re
                    m = re.search(r'lock=(\d+)', l); lock = int(m.group(1)) if m else 0
                    m = re.search(r'streak=(\d+)', l); streak = int(m.group(1)) if m else 0
                    m = re.search(r'qual=(\d+)', l); qual = int(m.group(1)) if m else 0
                    m = re.search(r'conf_inst=([-\d\.]+)', l); c_inst = float(m.group(1)) if m else 0.0
                    m = re.search(r'conf_win=([-\d\.]+)', l); c_win = float(m.group(1)) if m else 0.0
                    m = re.search(r'err=([-\d\.]+)deg', l); err_deg = float(m.group(1)) if m else 0.0
                    m = re.search(r'spd_err=([-\d\.]+)', l); spd_err = float(m.group(1)) if m else 0.0
                    m = re.search(r'win_rms=([-\d\.]+)', l); rms = float(m.group(1)) if m else 0.0
                    m = re.search(r'win_peak=([-\d\.]+)', l); peak = float(m.group(1)) if m else 0.0

            print(f"{t_curr:6.2f}|{vel:9.1f}|{lock:6d}|{c_inst:8.2f}|{c_win:7.2f}|{err_deg:9.1f}|{spd_err:8.1f}|{rms:9.1f}|{peak:7.1f}|{streak:8d}|{qual:8d}")
            time.sleep(0.3)

    finally:
        print("\n[步骤 4] 减速停机...")
        send_cmd(ser, "target 0")
        time.sleep(0.8)
        send_cmd(ser, "disable")
        ser.close()
        print("  [+] 停机完成，串口已关闭")

if __name__ == '__main__':
    main()
