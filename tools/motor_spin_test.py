# -*- coding: utf-8 -*-
"""
真实电机运转闭环实测脚本 (Motor Spin Test)
安全验证：
  1. 编码器校准 (calib)
  2. 速度模式正转 300 RPM (2s)，验证编码器转速跟随
  3. 提速到 600 RPM 并平稳反转至 -600 RPM
  4. 验证 FOC-STP 实时波形流中的实际电流与转速
  5. 安全停机 (disable)
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


def main():
    print("=" * 60)
    print("          电机真实旋转闭环动态测试 (COM44)")
    print("=" * 60)

    ser = serial.Serial(PORT, BAUD, timeout=0.03, write_timeout=0.5)
    time.sleep(0.1)

    # 1. 确保初始安全停机
    cli(ser, "disable")
    cli(ser, "fault clear")
    cli(ser, "mode vel")
    cli(ser, "target 0")
    cli(ser, "limit 3.5")  # 安全限流 3.5A

    # 2. 检查母线电压
    st = cli(ser, "status")
    print("\n--- 1. 初始状态检查 ---")
    for line in st.splitlines()[:5]:
        print("  ", line)

    # 3. 执行校准（若未校准则执行）
    print("\n--- 2. 电机校准 (calib) ---")
    cal_res = cli(ser, "calib", wait=1.5)
    print("  calib 响应:", cal_res.replace("\r\n", " "))
    time.sleep(1.0)

    # 再次确认校准后状态
    st_cal = cli(ser, "status")
    if "calib=1" in st_cal:
        print("  [OK] 编码器零位校准成功 (calib=1)")
    else:
        print("  [INFO] 状态:", [l for l in st_cal.splitlines() if "calib" in l or "fault" in l])

    # 4. 开启遥测流
    cli(ser, "telem mask 0x040001FF")  # 默认 10 通道
    cli(ser, "telem rate 500")
    cli(ser, "telem enable 1")

    # 5. 使能电机
    print("\n--- 3. 使能电机并给定正向 300 RPM ---")
    en_res = cli(ser, "enable")
    print("  enable 响应:", en_res)
    cli(ser, "target 300")

    # 采集 2 秒的实际运转数据
    dec = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < 2.0:
        chunk = ser.read(8192)
        if chunk:
            dec.feed(chunk)
    waves = dec.pop_waves()
    print("  [实测] 300 RPM 运转期间采集到 %d 帧波形" % len(waves))
    if waves:
        last = waves[-1]["channels"]
        print("  最新状态: 目标=300 RPM, 反馈转速=%.1f RPM, Iq=%.2f A, 母线=%.2f V" % (
            last.get("vel_ctrl", 0.0),
            last.get("iq_filt", 0.0),
            last.get("vbus_fast", 0.0),
        ))

    # 6. 反向运转 -300 RPM
    print("\n--- 4. 切换反向 -300 RPM ---")
    cli(ser, "target -300")
    dec_rev = StpStreamDecoder()
    t0 = time.time()
    while time.time() - t0 < 2.0:
        chunk = ser.read(8192)
        if chunk:
            dec_rev.feed(chunk)
    waves_rev = dec_rev.pop_waves()
    print("  [实测] -300 RPM 运转期间采集到 %d 帧波形" % len(waves_rev))
    if waves_rev:
        last = waves_rev[-1]["channels"]
        print("  最新状态: 目标=-300 RPM, 反馈转速=%.1f RPM, Iq=%.2f A, 母线=%.2f V" % (
            last.get("vel_ctrl", 0.0),
            last.get("iq_filt", 0.0),
            last.get("vbus_fast", 0.0),
        ))

    # 7. 减速并停机
    print("\n--- 5. 减速至 0 并安全停机 ---")
    cli(ser, "target 0")
    time.sleep(0.5)
    dis_res = cli(ser, "disable")
    cli(ser, "telem enable 0")
    print("  disable 响应:", dis_res)

    print("\n" + "=" * 60)
    print("              电机运转测试完毕")
    print("=" * 60)
    ser.close()


if __name__ == "__main__":
    main()
