# -*- coding: utf-8 -*-
"""
PB4 (RX) 物理引脚电平跳变侦听实验：
在 PC 往 COM44 持续发送交变字节 0x55 时，通过 SWD 高频采样 PB4 引脚电平，
判断 VLink TX 是否有任何电平物理送达到 STM32 的 PB4 引脚！
"""
import time
import serial
from pyocd.core.helpers import ConnectHelper

def test_pin_activity():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target
    if t.is_halted(): t.resume()

    print("=== 正在测试 VLink TX -> MCU PB4 (RX) 物理电平连续性 ===")

    # 打开 COM44，波特率设为较慢的 115200 以便 SWD 能够更容易捕获到方波低电平
    ser = serial.Serial("COM44", 115200, timeout=0.1)
    time.sleep(0.05)

    # 1. 静态未发送时采样 PB4
    static_levels = []
    for _ in range(50):
        idr = t.read32(0x48000400 + 0x10)
        static_levels.append((idr >> 4) & 1)
    print(f"1. 空闲状态 PB4 电平统计 (50 次): 1 的比例 = {static_levels.count(1)}/50, 0 的比例 = {static_levels.count(0)}/50")

    # 2. 持续发送 0x00 / 0x55 / Break，让 TX 产生大量的低电平
    print("2. 正在往 COM44 发送大块低电平/数据流...")
    # 发送长串 0x00 (在串口中 0x00 包含起始位0和8个数据位0，几乎全程是低电平！)
    ser.write(b"\x00" * 1000)
    ser.flush()

    active_levels = []
    t0 = time.time()
    while time.time() - t0 < 0.2:
        idr = t.read32(0x48000400 + 0x10)
        active_levels.append((idr >> 4) & 1)

    print(f"3. 发送期间 PB4 电平统计 ({len(active_levels)} 次采样): 1 的比例 = {active_levels.count(1)}, 0 的比例 = {active_levels.count(0)}")

    ser.close()
    session.close()

    if active_levels.count(0) > 0:
        print("\n>>> [结论]: 物理连线正常！PB4 成功捕获到了 VLink 发来的低电平跳变！")
    else:
        print("\n>>> [结论]: 物理断开！连续发送 1000 个 0x00 期间，PB4 引脚电平恒定为 1，完全没有任何电平变化！")
        print("          说明 VLink 的 TX 信号根本没有到达 STM32 的 PB4 引脚（杜邦线脱落、插偏或插反）！")

if __name__ == "__main__":
    test_pin_activity()
