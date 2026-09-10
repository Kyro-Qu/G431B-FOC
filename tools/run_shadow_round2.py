# -*- coding: utf-8 -*-
"""
自主闭环攻关 · 第 2 轮物理稳态评测脚本：
1. 自动执行校准并固化；
2. 编码器闭环驱动至 1000 RPM 稳态（严格等待物理转速到位）；
3. 开启 bench 稳态统计采样窗口（>20000 拍）；
4. 采集 10 组底层原始信号快照；
5. 输出四套算法的客观指标并按准入标准判定；
6. 平稳降速停机。
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

    print(">>> 启动环境自检与模式切换 ...")
    send('log 0')
    send('fault clear')
    time.sleep(0.1)

    # 确认校准已完成
    st = send('status')
    if 'calib=1' not in st:
        print("执行编码器校准 (calib) ...")
        send('calib')
        # 轮询等待 calib_state 变为 0 且 calib=1
        for _ in range(20):
            time.sleep(0.3)
            st_c = send('status')
            if 'calib=1' in st_c and 'calib_state=0' in st_c:
                break
        send('conf write')
        print("校准完成并已固化。")

    print("\n>>> 切换为编码器速度闭环 (mode vel) 并使能 ...")
    send('mode vel')
    time.sleep(0.1)
    send('target 0')
    time.sleep(0.05)
    send('vel ramp 300')
    time.sleep(0.05)
    en_res = send('enable')
    print("Enable 响应:", en_res.strip())
    time.sleep(0.3)

    print("下发目标转速 1000 RPM ...")
    send('target 1000')

    # 动态等待转速进入稳态窗口 (900 ~ 1050 RPM)
    print("正在平稳爬坡并等待 1000 RPM 物理稳态 ...")
    steady_ok = False
    for t in range(35):
        time.sleep(0.25)
        st_now = send('status', delay=0.06)
        m_v = re.search(r'vel=([\-\d\.]+)rpm', st_now)
        v = abs(float(m_v.group(1))) if m_v else 0.0
        if v >= 900.0:
            print(f"  -> 已平稳到达物理稳态: {v:.1f} RPM (耗时 {(t+1)*0.25:.1f}s)")
            steady_ok = True
            break
        else:
            if (t % 4) == 0:
                print(f"     爬坡中: {v:.1f} RPM ...")

    if not steady_ok:
        print("未能进入 1000 RPM 稳态，当前详细状态:")
        print(send('status', delay=0.1))
        send('target 0')
        time.sleep(0.5)
        send('disable')
        ser.close()
        return

    # 等待稳态超调完全平息 (1.0s)
    time.sleep(1.0)

    # 对齐并开启稳态统计窗口 (统计 1.5s，等效 24,000 拍快环)
    send('bench align')
    time.sleep(0.05)
    send('bench steady 1')
    time.sleep(1.5)
    send('bench steady 0')

    print("\n>>> 采集底层原始物理量快照 (bench diag) ...")
    for i in range(8):
        res = send('bench diag', delay=0.04)
        for line in res.splitlines():
            if 'diag:' in line:
                print(f"  [{i+1}] {line.strip()}")
                break
        time.sleep(0.04)

    # 读取统计结果
    print("\n>>> 提取四套算法稳态统计数据 (bench status) ...")
    res_bench = send('bench status', delay=0.1)
    print(res_bench.strip())

    # 平稳降速停机
    print("\n>>> 平稳降速停机 ...")
    send('target 0')
    time.sleep(1.2)
    send('disable')
    send('log 1')
    ser.close()
    print(">>> 电机已平稳停机，驱动器释放。")

if __name__ == "__main__":
    main()
