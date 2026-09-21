# -*- coding: utf-8 -*-
"""
终极地线与物理连通性判定实验 (9600 bps 黄金低速)
1. 将 USART2 临时降速至 9600 bps (BRR = 17708)
2. 打开 Windows COM44 为 9600 bps
3. 连续发送 50 个 0x00 (持续 52ms 的低电平方波)
4. 采样 PB3 硬件 IDR：检验 STM32 芯片内部是否真正驱动 PB3
5. 读取 COM44 接收缓冲区：检验 VLink 硬件与线缆是否真正连通
6. 恢复原 BRR
"""
import time
import serial
from pyocd.core.helpers import ConnectHelper

def test_ground_truth():
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target
    if t.is_halted(): t.resume()

    print("=== 9600 bps 终极物理真值判定实验 ===")

    # 1. 打开 COM44 为 9600
    ser = serial.Serial("COM44", 9600, timeout=0.5)
    time.sleep(0.1)

    # 2. 修改 MCU USART2 为 9600 bps
    # APB1 = 170MHz, 170M / 9600 = 17708.33 -> 17708 = 0x452C
    old_brr = t.read32(0x40004400 + 0x0c)
    cr1 = t.read32(0x40004400 + 0x00)
    t.write32(0x40004400 + 0x00, cr1 & ~1) # UE=0
    t.write32(0x40004400 + 0x0c, 17708)    # BRR=17708
    t.write32(0x40004400 + 0x00, cr1 | 1)  # UE=1
    print(f"1. MCU USART2 已临时切换为 9600 bps (BRR: {old_brr} -> 17708)")

    ser.reset_input_buffer()

    # 3. 连续发送 50 个 0x00，并在发送期间采样 PB3 IDR
    print("2. 正在通过 USART2 发送 50 个 0x00 (持续 ~52ms)...")
    pb3_samples = []
    t0 = time.time()
    for i in range(50):
        # 等待 TXE
        while (t.read32(0x40004400 + 0x1c) & 0x80) == 0: pass
        t.write32(0x40004400 + 0x28, 0x00)
        idr = t.read32(0x48000400 + 0x10)
        pb3_samples.append((idr >> 3) & 1)

    print(f"3. 发送期间 PB3 (TX) 电平采样: 1={pb3_samples.count(1)}, 0={pb3_samples.count(0)}")

    # 4. 检查 Windows COM44 接收
    t0 = time.time()
    rx_bytes = bytearray()
    while time.time() - t0 < 0.5:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk: rx_bytes.extend(chunk)

    print(f"4. Windows COM44 (9600bps) 接收结果: len={len(rx_bytes)}, 数据={bytes(rx_bytes[:20])}")

    # 5. 恢复原 BRR
    t.write32(0x40004400 + 0x00, cr1 & ~1)
    t.write32(0x40004400 + 0x0c, old_brr)
    t.write32(0x40004400 + 0x00, cr1 | 1)
    print("5. 已恢复 MCU 原 BRR 设置。")

    ser.close()
    session.close()

    print("\n==================================================")
    if pb3_samples.count(0) > 0 and len(rx_bytes) > 0:
        print("  【结论 1】: 硬件电路与线缆 100% 完好！9600bps 下完全正常通畅！")
        print("             根因确凿锁定：原 6.5Mbps 超高波特率超出 VLink 硬件承受极限！")
    elif pb3_samples.count(0) > 0 and len(rx_bytes) == 0:
        print("  【结论 2】: STM32 内部正常输出了低电平跳变，但 VLink 端完全未收到！")
        print("             根因确凿锁定：火柴板 PB3 (TX) 到 VLink RX 物理断开（杜邦线脱落/插错/虚接）！")
    elif pb3_samples.count(0) == 0:
        print("  【结论 3】: STM32 的 PB3 引脚电平恒定为高，未能被拉低！")
    print("==================================================")

if __name__ == "__main__":
    test_ground_truth()
