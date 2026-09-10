# -*- coding: utf-8 -*-
"""
执行编码器校准，并在 1000 RPM 闭环稳态下采集 15 组底层原始物理量快照 (bench diag) 与综合指标
"""
import sys
import serial
import time
import re

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')

PORT = "COM44"
BAUD = 6500000

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.15)
    except Exception as e:
        print(f"打开串口 {PORT} 失败: {e}")
        return

    ser.reset_input_buffer()

    def send(cmd, delay=0.08):
        ser.write((cmd + '\n').encode('ascii'))
        time.sleep(delay)
        if ser.in_waiting:
            return ser.read(ser.in_waiting).decode('ascii', errors='replace')
        return ''

    print("=== 步骤 1: 清除故障并执行编码器校准 (calib) ===")
    send('log 0')
    send('fault clear')
    res = send('calib', delay=0.3)
    print("下发校准响应:", res.strip())

    calib_ok = False
    for i in range(25):
        time.sleep(0.4)
        st = send('status', delay=0.08)
        if 'calib=1' in st:
            print(f"-> [第 {i+1} 轮检测] 编码器校准成功！(calib=1)")
            calib_ok = True
            break
        else:
            m_dir = re.search(r'calib_dir=([\-\d]+)', st)
            m_state = re.search(r'calib_state=(\d+)', st)
            print(f"   校准进行中... state={m_state.group(1) if m_state else '?'} dir={m_dir.group(1) if m_dir else '?'}")

    if not calib_ok:
        print("校准未完成，当前状态:")
        print(send('status', delay=0.1))
        ser.close()
        return

    print("\n=== 步骤 2: 将校准参数固化至 Flash (conf write) ===")
    cw = send('conf write', delay=0.2)
    print("Flash 固化响应:", cw.strip())

    print("\n=== 步骤 3: 编码器闭环恒速控制 (mode vel @ 1000 RPM) ===")
    send('mode vel')
    send('target 0')
    send('enable')
    time.sleep(0.3)
    send('target 1000')

    print("正在平稳爬坡至 1000 RPM 稳态 (等待 3.0 秒) ...")
    time.sleep(3.0)

    st_run = send('status', delay=0.1)
    for l in st_run.splitlines():
        if any(k in l for k in ['vel=', 'id=', 'iq=', 'vbus=', 'cpu=']):
            print("  运行状态:", l.strip())

    # 先对齐一次零点偏移，方便对比波动与绝对值
    send('bench align')
    send('bench steady 1')
    time.sleep(1.0)
    send('bench steady 0')

    print("\n=== 步骤 4: 采集 15 组底层原始物理量快照 (bench diag) ===")
    print("v_ab=(vα,vβ) | i_ab=(iα,iβ) | eta=(ηα,ηβ) | θ_raw vs θ_enc | bemf=(eα,eβ)")
    print("-" * 95)
    for i in range(15):
        res = send('bench diag', delay=0.04)
        for line in res.splitlines():
            if 'diag:' in line:
                print(f"[{i+1:02d}] {line.strip()}")
                break
        time.sleep(0.04)

    print("\n=== 步骤 5: 读取 1000 RPM 稳态下的四种观测器综合指标 ===")
    bench_st = send('bench status', delay=0.1)
    print(bench_st.strip())

    print("\n=== 步骤 6: 平稳降速停机 ===")
    send('target 0')
    time.sleep(1.0)
    send('disable')
    send('log 1')
    ser.close()
    print(">>> 测试安全结束，电机已完全停机。")

if __name__ == "__main__":
    main()
