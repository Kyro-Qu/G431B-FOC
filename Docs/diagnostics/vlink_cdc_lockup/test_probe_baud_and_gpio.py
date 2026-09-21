# -*- coding: utf-8 -*-
"""
核心证伪实验：
1. 保持当前 MCU 运行
2. 打开 Windows COM44 为 115200 波特率（此时 Windows 会下发 SET_LINE_CODING 115200 给 VLink）
3. 临时将 STM32 USART2 BRR 修改为 115200 (170MHz / 115200 = 1476 = 0x05C4)
4. 通过 TDR 发送 0x41 ('A'), 0x42 ('B'), 0x43 ('C'), 0x0A ('\n')
5. 观察 Windows COM44 能否收到数据！
6. 观察 ClearCommError 的状态变化！
"""
import time
import serial
from pyocd.core.helpers import ConnectHelper

def test_115200_transmission():
    print("=== 实验 1: 动态降速至 115200 bps 探测 VLink 接收 ===")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target
    if t.is_halted(): t.resume()

    # 打开 COM44 为 115200，触发 VLink 内部重新配置其 UART 硬件波特率
    print("1. 打开 COM44 为 115200...")
    ser = serial.Serial("COM44", 115200, timeout=0.5)
    time.sleep(0.1)

    # 读取当前 USART2 BRR
    old_brr = t.read32(0x40004400 + 0x0c)
    print(f"2. MCU 当前 USART2->BRR = 0x{old_brr:08x} ({old_brr})")

    # 改为 115200: 170M / 115200 = 1476 = 0x05C4
    # 修改 BRR 前需要关闭 UE (CR1 bit 0)
    cr1 = t.read32(0x40004400 + 0x00)
    t.write32(0x40004400 + 0x00, cr1 & ~1) # Disable USART2
    t.write32(0x40004400 + 0x0c, 1476)     # BRR = 1476
    t.write32(0x40004400 + 0x00, cr1 | 1)  # Enable USART2
    new_brr = t.read32(0x40004400 + 0x0c)
    print(f"3. MCU 已修改 USART2->BRR = 0x{new_brr:08x} ({new_brr}) [115200 bps]")

    # 清空上位机接收缓冲区
    ser.reset_input_buffer()

    # 通过 USART2->TDR 发送测试字节: "TEST\r\n"
    print("4. MCU 正在通过 TDR 发送 'TEST\\r\\n'...")
    for b in b"TEST\r\n":
        # 等待 TXE
        t0 = time.time()
        while (t.read32(0x40004400 + 0x1c) & 0x80) == 0:
            if time.time() - t0 > 0.1: break
        t.write32(0x40004400 + 0x28, b) # 写入 TDR
        time.sleep(0.001)

    # 检查 Windows 侧是否有数据收到
    t0 = time.time()
    rx_bytes = bytearray()
    while time.time() - t0 < 0.5:
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            rx_bytes.extend(chunk)
            break

    print(f"5. Windows COM44 接收结果: len={len(rx_bytes)}, data={bytes(rx_bytes)}")

    # 恢复原 BRR
    t.write32(0x40004400 + 0x00, cr1 & ~1)
    t.write32(0x40004400 + 0x0c, old_brr)
    t.write32(0x40004400 + 0x00, cr1 | 1)
    print(f"6. 已恢复 MCU 原 BRR = {old_brr}")

    ser.close()
    session.close()

if __name__ == "__main__":
    test_115200_transmission()
