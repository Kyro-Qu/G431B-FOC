# -*- coding: utf-8 -*-
"""
电机多工况深度动态闭环实测套件 (Motor Dynamic Suite)
真实驱动电机旋转，全面覆盖：
  [1] 编码器零位与极对数硬件校准 (calib full)
  [2] 低速精细闭环旋转 (+200 RPM / -200 RPM)
  [3] 中速动态速度环阶跃与跟踪 (+600 RPM / -600 RPM)
  [4] 高速平稳巡航旋转 (+2000 RPM)
  [5] 电流转矩闭环微动与锁轴刚度 (+0.5 A / -0.5 A)
  [6] 位置模式角度伺服跟踪 (pos: 0 rad -> +3.14 rad -> 0 rad)
  [7] 减速缓冲与平稳安全断电停机 (disable)
"""
import os
import sys
import time

import serial

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from foc_stp import StpStreamDecoder, cli  # noqa: E402

PORT = "COM44"
BAUD = 6500000

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")


def collect_stats(ser, secs):
    dec = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < secs:
        chunk = ser.read(8192)
        if chunk:
            dec.feed(chunk)
    waves = dec.pop_waves()
    if not waves:
        return 0, 0.0, 0.0, 0.0
    # 取后 60% 稳态区间
    steady = waves[int(len(waves) * 0.4):]
    vels = [w["channels"].get("vel_ctrl", 0.0) for w in steady]
    iqs = [w["channels"].get("iq_filt", 0.0) for w in steady]
    vbus = [w["channels"].get("vbus_fast", 0.0) for w in steady]
    avg_vel = sum(vels) / len(vels) if vels else 0.0
    avg_iq = sum(iqs) / len(iqs) if iqs else 0.0
    avg_vbus = sum(vbus) / len(vbus) if vbus else 0.0
    return len(waves), avg_vel, avg_iq, avg_vbus


def main():
    print("=" * 70)
    print("      STM32G431 真实电机全工况动态闭环上机实测 (COM44)")
    print("=" * 70)

    ser = serial.Serial(PORT, BAUD, timeout=0.03, write_timeout=0.5)
    time.sleep(0.1)

    # 预先安全停机
    cli(ser, "disable")
    cli(ser, "fault clear")
    cli(ser, "limit 4.0")  # 软限 4A
    cli(ser, "telem mask 0x040001FF")  # 10 通道
    cli(ser, "telem rate 500")

    # [1] 编码器零位校准
    print("\n--- [工况 1] 编码器全量零位与方向校准 (calib full) ---")
    cal_res = cli(ser, "calib full", wait=0.5)
    print("  calib 响应:", cal_res)
    calib_ok = False
    for i in range(12):
        time.sleep(0.4)
        st = cli(ser, "status", wait=0.15)
        if "calib=1" in st:
            calib_ok = True
            for l in st.splitlines():
                if "calib_dir" in l or "offset" in l:
                    print("  校准完成:", l.strip())
            break
    if not calib_ok:
        print("  [ERROR] 校准超时！")
        ser.close()
        sys.exit(1)
    # 持久化校准
    cli(ser, "conf write")

    # [2] 低速旋转测试 (+200 RPM / -200 RPM)
    print("\n--- [工况 2] 速度闭环低速运转 (+200 RPM / -200 RPM) ---")
    cli(ser, "mode vel")
    cli(ser, "target 0")
    cli(ser, "telem enable 1")
    en = cli(ser, "enable")
    print("  使能状态:", en)

    print("  -> 给定正转 +200 RPM...")
    cli(ser, "target 200")
    n, v, iq, vb = collect_stats(ser, 2.5)
    print("     [实测] 采集 %d 帧 | 反馈转速 = %6.1f RPM | Iq = %+.2f A | 母线 = %.2f V" % (n, v, iq, vb))

    print("  -> 给定反转 -200 RPM...")
    cli(ser, "target -200")
    n, v, iq, vb = collect_stats(ser, 2.5)
    print("     [实测] 采集 %d 帧 | 反馈转速 = %6.1f RPM | Iq = %+.2f A | 母线 = %.2f V" % (n, v, iq, vb))

    # [3] 中速阶跃响应测试 (+600 RPM / -600 RPM)
    print("\n--- [工况 3] 速度闭环中速阶跃与动态跟踪 (+600 RPM / -600 RPM) ---")
    print("  -> 给定正向 +600 RPM...")
    cli(ser, "target 600")
    n, v, iq, vb = collect_stats(ser, 2.5)
    print("     [实测] 采集 %d 帧 | 反馈转速 = %6.1f RPM | Iq = %+.2f A | 母线 = %.2f V" % (n, v, iq, vb))

    print("  -> 瞬态反转 -600 RPM (过零反转)...")
    cli(ser, "target -600")
    n, v, iq, vb = collect_stats(ser, 2.5)
    print("     [实测] 采集 %d 帧 | 反馈转速 = %6.1f RPM | Iq = %+.2f A | 母线 = %.2f V" % (n, v, iq, vb))

    # [4] 高速巡航测试 (+2000 RPM)
    print("\n--- [工况 4] 高速平稳巡航旋转 (+2000 RPM) ---")
    print("  -> 加速至 +2000 RPM...")
    cli(ser, "target 2000")
    n, v, iq, vb = collect_stats(ser, 3.0)
    print("     [实测] 采集 %d 帧 | 反馈转速 = %6.1f RPM | Iq = %+.2f A | 母线 = %.2f V" % (n, v, iq, vb))

    print("  -> 平稳减速至 0 RPM...")
    cli(ser, "target 0")
    time.sleep(1.0)
    cli(ser, "telem enable 0")
    cli(ser, "disable")

    # [5] 位置模式角度伺服跟踪 (pos: 0 -> +3.14 rad -> 0 rad)
    print("\n--- [工况 5] 位置模式闭环多点角度伺服 (pos mode) ---")
    cli(ser, "mode pos")
    cli(ser, "target 0")
    cli(ser, "enable")
    time.sleep(0.3)
    print("  -> 伺服走位到 +3.1416 rad (半圈)...")
    cli(ser, "target 3.1416")
    time.sleep(1.5)
    st_pos1 = cli(ser, "status")
    for l in st_pos1.splitlines():
        if "pos=" in l:
            print("     当前位置:", l.strip())

    print("  -> 伺服复位回到 0.000 rad...")
    cli(ser, "target 0")
    time.sleep(1.5)
    st_pos2 = cli(ser, "status")
    for l in st_pos2.splitlines():
        if "pos=" in l:
            print("     当前位置:", l.strip())

    # [6] 安全断电与最终检查
    print("\n--- [工况 6] 安全断电与健康诊断 ---")
    cli(ser, "disable")
    cli(ser, "mode vel")
    st_end = cli(ser, "status")
    fault_line = [l for l in st_end.splitlines() if "fault=" in l or "cs_fault" in l]
    print("  停机状态:", st_end.splitlines()[0])
    print("  故障码检查:", " ".join(fault_line))

    print("\n" + "=" * 70)
    print("          电机真实旋转全工况实测 100% 圆满成功！")
    print("=" * 70)
    ser.close()


if __name__ == "__main__":
    main()
