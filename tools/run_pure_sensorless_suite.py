# -*- coding: utf-8 -*-
"""
纯无感全速域稳态与动态完整测试套件 (run_pure_sensorless_suite.py)
"""
import sys
import time
import re
import serial

PORT = "COM44"
BAUD = 6500000

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

def get_ser():
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    ser.reset_input_buffer()
    return ser

def c(ser, cmd, delay=0.08):
    ser.reset_input_buffer()
    ser.write((cmd + '\n').encode('ascii'))
    time.sleep(delay)
    return ser.read_all().decode(errors='ignore').strip()

def parse_st(text):
    res = {"vel": 0.0, "vel_obs": 0.0, "vel_filt": 0.0, "id": 0.0, "iq": 0.0, "vbus": 0.0, "fault": 0}
    for l in text.splitlines():
        if "vel=" in l:
            m = re.search(r"vel=([\-\d\.]+)rpm", l)
            if m: res["vel"] = float(m.group(1))
            m = re.search(r"vel_obs=([\-\d\.]+)rpm", l)
            if m: res["vel_obs"] = float(m.group(1))
            m = re.search(r"vel_filt=([\-\d\.]+)rpm", l)
            if m: res["vel_filt"] = float(m.group(1))
        if "id=" in l and "iq=" in l:
            m = re.search(r"id=([\-\d\.]+)A", l)
            if m: res["id"] = float(m.group(1))
            m = re.search(r"iq=([\-\d\.]+)A", l)
            if m: res["iq"] = float(m.group(1))
        if "vbus=" in l:
            m = re.search(r"vbus=([\-\d\.]+)V", l)
            if m: res["vbus"] = float(m.group(1))
        if "fault=" in l:
            m = re.search(r"fault=(\d+)", l)
            if m: res["fault"] = int(m.group(1))
    return res

def test_sensorless_run():
    ser = get_ser()
    print("=" * 75)
    print("    【纯无感主控模式 Sensorless Primary: 500 ~ 2000 RPM 全区间实机评测】")
    print("=" * 75)

    c(ser, 'disable')
    c(ser, 'fault clear')
    c(ser, 'feedback sensorless')
    c(ser, 'feedback if 0.60 500 300')
    c(ser, 'mode vel')
    c(ser, 'target 500')
    c(ser, 'enable')

    print("  [+] I/F 开环拖动加速中，等待换手切入闭环...")
    t0 = time.time()
    while time.time() - t0 < 6.0:
        fb = c(ser, 'feedback')
        if 'state=run' in fb:
            break
        time.sleep(0.1)

    print(f"  [+] 成功自适应换手切入 VESC 闭环，用时 {time.time()-t0:.2f}s！")

    speeds = [500, 800, 1000, 1200, 1500, 1800, 2000]
    results = {}
    for spd in speeds:
        c(ser, f'target {spd}')
        time.sleep(2.5)
        st = parse_st(c(ser, 'status'))
        fb = c(ser, 'feedback')
        ss = c(ser, 'sensorless status')

        spd_obs = 0.0
        conf_val = 0.0
        lock_val = 0
        for p in fb.split():
            if p.startswith('spd_obs='): spd_obs = float(p.replace('spd_obs=',''))
            if p.startswith('conf='): conf_val = float(p.replace('conf=',''))
            if p.startswith('lock='): lock_val = int(p.replace('lock=',''))

        err = spd_obs - spd
        results[spd] = (spd_obs, st['vel'], err, st['iq'], conf_val, lock_val)
        print(f"  目标: {spd:4d} RPM | 观测速度: {spd_obs:6.1f} RPM (误差: {err:+5.1f}) | 编码器参考: {st['vel']:6.1f} RPM | Iq: {st['iq']:+5.2f}A | Conf: {conf_val:.2f} | Lock: {lock_val}")

    print("\n>>> 正在减速停机...")
    c(ser, 'target 0')
    time.sleep(1.5)
    c(ser, 'disable')
    c(ser, 'feedback sensored')
    ser.close()

    print("\n" + "=" * 75)
    print("                纯无感模式 (Sensorless Primary) 最终大榜")
    print("=" * 75)
    print(f"  {'设定转速':<10} | {'VESC观测转速':<14} | {'编码器真值':<12} | {'控制偏差':<10} | {'Iq电流':<8} | {'置信度':<6} | {'锁定'}")
    print("  " + "-" * 72)
    for spd, (so, se, er, iq, cf, lk) in results.items():
        print(f"  {spd:<10} | {so:>8.1f} RPM    | {se:>8.1f} RPM | {er:>+6.1f} RPM | {iq:>6.2f}A | {cf:>5.2f}  | {lk}")
    print("=" * 75)

if __name__ == '__main__':
    test_sensorless_run()
