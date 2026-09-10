# -*- coding: utf-8 -*-
"""
测量 VESC 原始观测器电角度 (未经 advance_rad 修正前)
与真实编码器电角度在正转与反转各转速下的真实差值
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

def measure_raw():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.05)
    except Exception as e:
        print(f"Error opening port: {e}")
        return

    time.sleep(0.1)

    def send(cmd, delay=0.03):
        ser.write((cmd + '\r\n').encode('ascii'))
        time.sleep(delay)
        buf = ''
        if ser.in_waiting:
            buf = ser.read(ser.in_waiting).decode('ascii', errors='ignore')
        return buf

    send('fault clear')
    send('enc fault clear')
    send('sensorless algo vesc')
    send('deadtime obs 1')
    send('deadtime volt 0.17')
    send('mode vel')
    send('vel ramp 800')
    send('enable')

    speeds = [800, 1000, 1200, 1400, 1600, 1800, -800, -1000, -1200, -1400, -1600, -1800]
    print(f"{'Target RPM':>10} | {'Actual RPM':>10} | {'Current err(°)':>15} | {'Raw Diff(rad)':>15} | {'Raw Diff(°)':>15}")
    print("-" * 75)

    for sp in speeds:
        send(f'target {sp}')
        time.sleep(1.5)
        st = send('status', 0.03)
        sl = send('sensorless status', 0.03)
        m_v = re.search(r'vel=([-\d\.]+)rpm', st)
        m_e = re.search(r'err=([-\d\.]+)deg', sl)
        vel = float(m_v.group(1)) if m_v else 0.0
        err = float(m_e.group(1)) if m_e else 0.0

        # 当前固件中加的补偿是: 0.109f + dir_blend * 0.682f
        dir_blend = 1.0 if vel > 150.0 else (-1.0 if vel < -150.0 else vel / 150.0)
        curr_adv = 0.109 + (dir_blend * 0.682)
        # err = (th_obs_aligned - th_enc) = (th_raw + curr_adv - th_enc)
        # 所以 raw_diff = (th_raw - th_enc) = err_rad - curr_adv
        err_rad = err * math.pi / 180.0
        raw_diff_rad = err_rad - curr_adv
        # wrap 到 [-pi, pi]
        while raw_diff_rad > math.pi: raw_diff_rad -= 2*math.pi
        while raw_diff_rad < -math.pi: raw_diff_rad += 2*math.pi

        raw_diff_deg = raw_diff_rad * 180.0 / math.pi
        print(f"{sp:10d} | {vel:10.1f} | {err:15.1f} | {raw_diff_rad:15.3f} | {raw_diff_deg:15.1f}")

    send('target 0')
    time.sleep(0.8)
    send('disable')
    ser.close()

if __name__ == '__main__':
    measure_raw()
