# -*- coding: utf-8 -*-
"""
tools/measure_speed_phase_curve.py
测量 500~1800 RPM (正反向) 各转速下 VESC 原始观测器与物理编码器的真实相角差 (精确稳态采样)
"""
import sys
import serial
import time
import math
import re

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    time.sleep(0.1)

    def send(cmd, delay=0.04):
        ser.reset_input_buffer()
        ser.write((cmd + '\r\n').encode('ascii'))
        time.sleep(delay)
        buf = ''
        while ser.in_waiting:
            buf += ser.read(ser.in_waiting).decode('ascii', errors='ignore')
            time.sleep(0.005)
        return buf.strip()

    send('fault clear')
    send('enc fault clear')
    send('sensorless algo vesc')
    send('deadtime obs 1')
    send('deadtime volt 0.17')
    send('mode vel')
    send('vel ramp 800')
    send('enable')

    test_speeds = [600, 700, 800, 900, 1000, 1200, 1500, 1800,
                   -600, -700, -800, -900, -1000, -1200, -1500, -1800]

    print(f"{'Target':>8} | {'Vel':>8} | {'Err(deg)':>10} | {'Raw Diff(rad)':>15} | {'Raw Diff(deg)':>15} | {'Needed Adv(rad)':>16}")
    print("-" * 85)

    for sp in test_speeds:
        send(f'target {sp}')
        # 等待平稳加速到目标转速
        time.sleep(2.0)
        # 采集 3 次取平均
        err_list = []
        vel_list = []
        for _ in range(4):
            time.sleep(0.1)
            st = send('status')
            sl = send('sensorless status')
            m_v = re.search(r'vel=([-\d\.]+)rpm', st)
            m_e = re.search(r'err=([-\d\.]+)deg', sl)
            if m_v and m_e:
                vel_list.append(float(m_v.group(1)))
                err_list.append(float(m_e.group(1)))

        if vel_list:
            vel = sum(vel_list) / len(vel_list)
            err = sum(err_list) / len(err_list)
            dir_blend = 1.0 if vel > 150.0 else (-1.0 if vel < -150.0 else vel / 150.0)
            curr_adv = 0.109 + (dir_blend * 0.682)
            err_rad = err * math.pi / 180.0
            raw_diff_rad = err_rad - curr_adv
            while raw_diff_rad > math.pi: raw_diff_rad -= 2*math.pi
            while raw_diff_rad < -math.pi: raw_diff_rad += 2*math.pi
            raw_diff_deg = raw_diff_rad * 180.0 / math.pi
            # 理想超前补偿量 = -raw_diff_rad (即消除 raw_diff)
            needed_adv_rad = -raw_diff_rad
            while needed_adv_rad > math.pi: needed_adv_rad -= 2*math.pi
            while needed_adv_rad < -math.pi: needed_adv_rad += 2*math.pi

            print(f"{sp:8d} | {vel:8.1f} | {err:10.2f} | {raw_diff_rad:15.3f} | {raw_diff_deg:15.2f} | {needed_adv_rad:16.3f}")

    send('target 0')
    time.sleep(1.0)
    send('disable')
    ser.close()

if __name__ == '__main__':
    main()
