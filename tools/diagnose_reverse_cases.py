# -*- coding: utf-8 -*-
"""
反向动态边界工况专项高频诊断脚本
针对两项工况:
  1. 反向加速 (-800 RPM/s) 注入 FREEZE
  2. 反向减速 (+800 RPM/s) 注入 STEP (+60°)
抓取:
  theta_encoder, theta_obs, speed_encoder, speed_obs,
  handover_delta, blend, conf, lock, iq, qualified_cycles
"""
import sys
import serial
import time
import re

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def run_diag():
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

    def query_all():
        st = send('status', 0.02)
        fb = send('feedback', 0.02)
        sl = send('sensorless status', 0.02)
        return st + '\n' + fb + '\n' + sl

    # 1. 启动配置
    send('fault clear')
    send('enc fault clear')
    send('sensorless algo vesc')
    send('deadtime obs 1')
    send('deadtime volt 0.17')
    send('feedback auto')
    send('mode vel')
    send('vel ramp 800')

    print("\n" + "="*80)
    print(">>> 诊断工况 1: 反向加速 (-800 RPM/s) 注入 FREEZE <<<")
    print("="*80)
    send('target -800')
    send('enable')
    time.sleep(2.0)

    # 打印 -800 RPM 稳态基准
    raw = query_all()
    print("--- [-800 RPM 稳态基准] ---")
    for l in raw.splitlines():
        if any(k in l for k in ['vel=', 'iq=', 'state=', 'sensorless:']):
            print("  ", l)

    # 发起加速到 -1800，监控并在 -1200 时注入
    send('target -1800')
    t0 = time.time()
    injected = False
    log1 = []
    while time.time() - t0 < 3.5:
        txt = query_all()
        log1.append((time.time() - t0, txt))
        # 找转速
        m = re.search(r'vel=([-\d\.]+)rpm', txt)
        if m and not injected:
            v = float(m.group(1))
            if v <= -1200.0:
                send('enc fault freeze')
                injected = True
                print(f"  [⚡ 注入时刻 FREEZE] v={v:.1f} RPM")
        time.sleep(0.04)

    send('enc fault clear')
    time.sleep(0.8)
    send('target 0')
    time.sleep(0.8)

    print("\n" + "="*80)
    print(">>> 诊断工况 2: 反向减速 (+800 RPM/s) 注入 STEP (+60°) <<<")
    print("="*80)
    send('target -1800')
    time.sleep(2.5)

    raw = query_all()
    print("--- [-1800 RPM 稳态基准] ---")
    for l in raw.splitlines():
        if any(k in l for k in ['vel=', 'iq=', 'state=', 'sensorless:']):
            print("  ", l)

    # 发起减速到 -800，监控并在 -1300 时注入
    send('target -800')
    t0 = time.time()
    injected = False
    log2 = []
    while time.time() - t0 < 3.5:
        txt = query_all()
        log2.append((time.time() - t0, txt))
        m = re.search(r'vel=([-\d\.]+)rpm', txt)
        if m and not injected:
            v = float(m.group(1))
            if v >= -1300.0: # 从 -1800 往 -800 减速，值从负大变负小
                send('enc fault step 60')
                injected = True
                print(f"  [⚡ 注入时刻 STEP 60] v={v:.1f} RPM")
        time.sleep(0.04)

    send('enc fault clear')
    time.sleep(0.8)
    send('target 0')
    time.sleep(0.8)
    send('disable')

    ser.close()

    # 分析工况 1 轨迹
    print("\n" + "-"*80)
    print("工况 1 轨迹分析 (-800 -> -1800):")
    for t, txt in log1:
        v = re.search(r'vel=([-\d\.]+)rpm', txt)
        st = re.search(r'state=(\w+)', txt)
        bl = re.search(r'blend=([-\d\.]+)', txt)
        err = re.search(r'err=([-\d\.]+)deg', txt)
        spderr = re.search(r'spd_err=([-\d\.]+)', txt)
        lck = re.search(r'lock=(\d+)', txt)
        cnf = re.search(r'conf=([-\d\.]+)', txt)
        q = re.search(r'qual=(\d+)', txt)
        iq = re.search(r'iq=([-\d\.]+)A', txt)
        if v and err:
            print(f"t={t:5.2f}s | vel={float(v.group(1)):7.1f} | iq={float(iq.group(1)):5.2f} | state={st.group(1) if st else '?':10} | blend={float(bl.group(1)) if bl else 0:4.2f} | err={float(err.group(1)):6.1f}° | spd_err={float(spderr.group(1)) if spderr else 0:4.0f} | lck={lck.group(1) if lck else '?'} | cnf={float(cnf.group(1)) if cnf else 0:4.2f} | qual={q.group(1) if q else '?'}")

    # 分析工况 2 轨迹
    print("\n" + "-"*80)
    print("工况 2 轨迹分析 (-1800 -> -800):")
    for t, txt in log2:
        v = re.search(r'vel=([-\d\.]+)rpm', txt)
        st = re.search(r'state=(\w+)', txt)
        bl = re.search(r'blend=([-\d\.]+)', txt)
        err = re.search(r'err=([-\d\.]+)deg', txt)
        spderr = re.search(r'spd_err=([-\d\.]+)', txt)
        lck = re.search(r'lock=(\d+)', txt)
        cnf = re.search(r'conf=([-\d\.]+)', txt)
        q = re.search(r'qual=(\d+)', txt)
        iq = re.search(r'iq=([-\d\.]+)A', txt)
        if v and err:
            print(f"t={t:5.2f}s | vel={float(v.group(1)):7.1f} | iq={float(iq.group(1)):5.2f} | state={st.group(1) if st else '?':10} | blend={float(bl.group(1)) if bl else 0:4.2f} | err={float(err.group(1)):6.1f}° | spd_err={float(spderr.group(1)) if spderr else 0:4.0f} | lck={lck.group(1) if lck else '?'} | cnf={float(cnf.group(1)) if cnf else 0:4.2f} | qual={q.group(1) if q else '?'}")

if __name__ == '__main__':
    run_diag()
