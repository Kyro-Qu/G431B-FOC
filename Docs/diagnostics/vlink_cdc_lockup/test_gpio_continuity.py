# -*- coding: utf-8 -*-
"""
实验 2: PB3 电气与中断连续性探测
测试 PB3 切换为 GPIO 输出，模拟 Break 信号，检查 Windows 驱动端是否感知到 CE_BREAK
"""
import time
import serial
import ctypes
from ctypes import wintypes
from pyocd.core.helpers import ConnectHelper

class COMSTAT(ctypes.Structure):
    _fields_ = [
        ('fCflags', wintypes.DWORD),
        ('cbInQue', wintypes.DWORD),
        ('cbOutQue', wintypes.DWORD)
    ]

def test_break_signal():
    print("=== 实验 2: PB3 模拟 Break 信号与电气测试 ===")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target
    if t.is_halted(): t.resume()

    ser = serial.Serial("COM44", 115200, timeout=0.1)
    errors = wintypes.DWORD()
    comstat = COMSTAT()
    ctypes.windll.kernel32.ClearCommError(ser._port_handle, ctypes.byref(errors), ctypes.byref(comstat))
    print(f"初始状态: Errors=0x{errors.value:08x}, InQue={comstat.cbInQue}")

    # 保存 GPIOB MODER 和 ODR
    moder = t.read32(0x48000400 + 0x00)
    odr = t.read32(0x48000400 + 0x14)

    print("1. 将 PB3 配置为 GPIO 输出 (Output Push-Pull)...")
    # PB3 mode 是 moder 的 bit 7:6。00=Input, 01=Output, 10=AF, 11=Analog
    # 清除 bit 7:6 并设为 01
    moder_out = (moder & ~(3 << 6)) | (1 << 6)
    t.write32(0x48000400 + 0x00, moder_out)

    print("2. 拉低 PB3 (产生 Line Break 信号，持续 100ms)...")
    t.write32(0x48000400 + 0x14, odr & ~(1 << 3)) # PB3 = 0
    time.sleep(0.1)

    print("3. 拉高 PB3 (恢复空闲高电平)...")
    t.write32(0x48000400 + 0x14, odr | (1 << 3))  # PB3 = 1
    time.sleep(0.05)

    # 检查 Windows 端是否收到了任何数据或 Break 错误
    ctypes.windll.kernel32.ClearCommError(ser._port_handle, ctypes.byref(errors), ctypes.byref(comstat))
    rx = ser.read(ser.in_waiting or 1)
    print(f"4. 经过 Break 后 Windows 响应: Errors=0x{errors.value:08x} (CE_BREAK={bool(errors.value & 0x10)}, CE_FRAME={bool(errors.value & 0x08)}), InQue={comstat.cbInQue}, rx_bytes={rx}")

    print("5. 恢复 PB3 为 AF 复用功能...")
    t.write32(0x48000400 + 0x00, moder)

    ser.close()
    session.close()

if __name__ == "__main__":
    test_break_signal()
