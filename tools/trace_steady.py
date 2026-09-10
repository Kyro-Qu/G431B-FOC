# -*- coding: utf-8 -*-
"""
tools/trace_steady.py
观察 800 RPM 稳态运行 10 秒时的 C_Win, C_Inst, Streak, Qual 建立全过程
"""
import sys
import time
import serial
import re

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
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except Exception as e:
        print(f"[-] 打开串口 {PORT} 失败: {e}")
        return 1

    time.sleep(0.05)
    try:
        send_cmd(ser, "fault clear")
        send_cmd(ser, "enc fault clear")
        time.sleep(0.1)

        st = send_cmd(ser, "status")
        if "calib=1*" not in st:
            send_cmd(ser, "calib")
            for _ in range(40):
                time.sleep(0.3)
                st_w = send_cmd(ser, "status")
                if "calib=1*" in st_w and "M0 IDLE" in st_w:
                    break

        cmds = [
            "feedback auto",
            "feedback speed 420 320",
            "mode vel",
            "vel ramp 800",
            "target 800",
            "enable"
        ]
        for c in cmds:
            send_cmd(ser, c)

        print(f"{'Time':^6}|{'Vel':^7}|{'C_Inst':^7}|{'C_Win':^6}|{'Err':^7}|{'SpdErr':^7}|{'RMS':^6}|{'Peak':^6}|{'Streak':^7}|{'Streak(ms)':^10}|{'Qual':^6}|{'Fail':^10}")
        print("-" * 100)

        t0 = time.time()
        while time.time() - t0 < 8.0:
            t_curr = time.time() - t0
            st = send_cmd(ser, "status", 0.03)
            sl = send_cmd(ser, "sensorless status", 0.03)
            vel = 0.0
            streak = 0
            qual = 0
            c_inst = 0.0
            c_win = 0.0
            err_deg = 0.0
            spd_err = 0.0
            rms = 0.0
            peak = 0.0
            fail_str = "-"

            for l in st.splitlines():
                if l.startswith("vel="):
                    try: vel = float(l.split("=")[1].replace("rpm",""))
                    except: pass
            for l in sl.splitlines():
                if l.startswith("sensorless:"):
                    m = re.search(r'streak=(\d+)', l); streak = int(m.group(1)) if m else 0
                    m = re.search(r'qual=(\d+)', l); qual = int(m.group(1)) if m else 0
                    m = re.search(r'conf_inst=([-\d\.]+)', l); c_inst = float(m.group(1)) if m else 0.0
                    m = re.search(r'conf_win=([-\d\.]+)', l); c_win = float(m.group(1)) if m else 0.0
                    m = re.search(r'err=([-\d\.]+)deg', l); err_deg = float(m.group(1)) if m else 0.0
                    m = re.search(r'spd_err=([-\d\.]+)', l); spd_err = float(m.group(1)) if m else 0.0
                    m = re.search(r'win_rms=([-\d\.]+)', l); rms = float(m.group(1)) if m else 0.0
                    m = re.search(r'win_peak=([-\d\.]+)', l); peak = float(m.group(1)) if m else 0.0
                    m_f = re.search(r'strk_fail=(\d+)\(r=(\d+)\)', l)
                    if m_f:
                        fail_cnt = int(m_f.group(1))
                        fail_r = int(m_f.group(2))
                        # 1:conf,2:lock,3:rms,4:peak,5:spd_err,6:dir,7:flux,8:err_deg,9:spd_low
                        r_map = {1:"conf",2:"lock",3:"rms",4:"peak",5:"spd_err",6:"dir",7:"flux",8:"err_deg",9:"spd_low"}
                        fail_str = f"{fail_cnt}({r_map.get(fail_r, str(fail_r))})"

            print(f"{t_curr:6.2f}|{vel:7.1f}|{c_inst:7.2f}|{c_win:6.2f}|{err_deg:7.1f}|{spd_err:7.1f}|{rms:6.1f}|{peak:6.1f}|{streak:7d}|{streak/16:9.1f}ms|{qual:6d}|{fail_str:^10}")
            time.sleep(0.3)

    finally:
        send_cmd(ser, "target 0")
        time.sleep(0.8)
        send_cmd(ser, "disable")
        ser.close()

if __name__ == '__main__':
    main()
