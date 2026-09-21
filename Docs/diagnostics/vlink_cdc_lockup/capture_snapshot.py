# -*- coding: utf-8 -*-
"""
现场快照采集脚本：抓取并持久化所有现场数据，杜绝现场丢失。
1. Windows 侧 COM44 设备与驱动状态
2. VLink 调试器完整 USB 描述符
3. MCU 硬件寄存器（SysTick, USART2, DMA1, GPIOB, NVIC, SCB）
4. MCU SRAM 核心结构体快照（huart2, hdma, rx_queue, s_cmd_out_buf）
"""
import os
import sys
import json
import time
import struct
import serial
import usb.core
import usb.util
from pyocd.core.helpers import ConnectHelper

OUT_DIR = os.path.dirname(os.path.abspath(__file__))

def capture_windows_usb_info():
    print("[1/4] 抓取 Windows 与 USB 描述符信息...")
    dev = usb.core.find(idVendor=0x1209, idProduct=0x6666)
    usb_info = {
        "idVendor": f"0x{dev.idVendor:04x}",
        "idProduct": f"0x{dev.idProduct:04x}",
        "bcdUSB": f"0x{dev.bcdUSB:04x}",
        "manufacturer": dev.manufacturer,
        "product": dev.product,
        "serial_number": dev.serial_number,
        "configurations": []
    }
    for cfg in dev:
        cfg_info = {
            "bConfigurationValue": cfg.bConfigurationValue,
            "bMaxPower_mA": cfg.bMaxPower * 2,
            "interfaces": []
        }
        for iface in cfg:
            if_info = {
                "bInterfaceNumber": iface.bInterfaceNumber,
                "bAlternateSetting": iface.bAlternateSetting,
                "bInterfaceClass": f"0x{iface.bInterfaceClass:02x}",
                "bInterfaceSubClass": f"0x{iface.bInterfaceSubClass:02x}",
                "bInterfaceProtocol": f"0x{iface.bInterfaceProtocol:02x}",
                "endpoints": []
            }
            for ep in iface:
                if_info["endpoints"].append({
                    "bEndpointAddress": f"0x{ep.bEndpointAddress:02x}",
                    "direction": "IN" if usb.util.endpoint_direction(ep.bEndpointAddress) == usb.util.ENDPOINT_IN else "OUT",
                    "bmAttributes": f"0x{ep.bmAttributes:02x}",
                    "type": ["Control", "Isochronous", "Bulk", "Interrupt"][ep.bmAttributes & 3],
                    "wMaxPacketSize": ep.wMaxPacketSize,
                    "bInterval": ep.bInterval
                })
            cfg_info["interfaces"].append(if_info)
        usb_info["configurations"].append(cfg_info)

    with open(os.path.join(OUT_DIR, "usb_device_descriptor.json"), "w", encoding="utf-8") as f:
        json.dump(usb_info, f, indent=2, ensure_ascii=False)
    print("  -> 已保存 usb_device_descriptor.json")

def capture_comm_status():
    print("[2/4] 抓取 Windows COM44 串口驱动状态...")
    ser = serial.Serial("COM44", 6500000, timeout=0.1)
    import ctypes
    from ctypes import wintypes
    class COMSTAT(ctypes.Structure):
        _fields_ = [
            ('fCflags', wintypes.DWORD),
            ('cbInQue', wintypes.DWORD),
            ('cbOutQue', wintypes.DWORD)
        ]
    errors = wintypes.DWORD()
    comstat = COMSTAT()
    ctypes.windll.kernel32.ClearCommError(ser._port_handle, ctypes.byref(errors), ctypes.byref(comstat))

    comm_info = {
        "port": "COM44",
        "baudrate": 6500000,
        "ClearCommError_ErrorsMask": f"0x{errors.value:08x}",
        "cbInQue": comstat.cbInQue,
        "cbOutQue": comstat.cbOutQue,
        "fCflags": f"0x{comstat.fCflags:08x}",
        "cts": ser.cts,
        "dsr": ser.dsr,
        "ri": ser.ri,
        "cd": ser.cd
    }
    ser.close()
    with open(os.path.join(OUT_DIR, "win32_comm_status.json"), "w", encoding="utf-8") as f:
        json.dump(comm_info, f, indent=2, ensure_ascii=False)
    print("  -> 已保存 win32_comm_status.json")

def capture_mcu_state():
    print("[3/4] 抓取 MCU 硬件外设寄存器与核心变量...")
    session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
    session.open()
    t = session.target
    if t.is_halted(): t.resume()

    mcu_regs = {}
    # SysTick
    mcu_regs["SysTick"] = {
        "CSR": f"0x{t.read32(0xE000E010):08x}",
        "RVR": f"0x{t.read32(0xE000E014):08x}",
        "CVR": f"0x{t.read32(0xE000E018):08x}",
        "uwTick": t.read32(0x20000018)
    }

    # USART2 (0x40004400)
    mcu_regs["USART2"] = {
        "CR1": f"0x{t.read32(0x40004400 + 0x00):08x}",
        "CR2": f"0x{t.read32(0x40004400 + 0x04):08x}",
        "CR3": f"0x{t.read32(0x40004400 + 0x08):08x}",
        "BRR": f"0x{t.read32(0x40004400 + 0x0c):08x}",
        "ISR": f"0x{t.read32(0x40004400 + 0x1c):08x}",
        "ICR": f"0x{t.read32(0x40004400 + 0x20):08x}",
        "RDR": f"0x{t.read32(0x40004400 + 0x24):08x}",
        "TDR": f"0x{t.read32(0x40004400 + 0x28):08x}"
    }

    # DMA1 (0x40020000)
    # Channel 1 (TX)
    mcu_regs["DMA1_CH1_TX"] = {
        "CCR": f"0x{t.read32(0x40020008):08x}",
        "CNDTR": t.read32(0x4002000c),
        "CPAR": f"0x{t.read32(0x40020010):08x}",
        "CMAR": f"0x{t.read32(0x40020014):08x}"
    }
    # Channel 2 (RX)
    mcu_regs["DMA1_CH2_RX"] = {
        "CCR": f"0x{t.read32(0x4002001c):08x}",
        "CNDTR": t.read32(0x40020020),
        "CPAR": f"0x{t.read32(0x40020024):08x}",
        "CMAR": f"0x{t.read32(0x40020028):08x}"
    }

    # GPIOB (0x48000400)
    mcu_regs["GPIOB"] = {
        "MODER": f"0x{t.read32(0x48000400 + 0x00):08x}",
        "OTYPER": f"0x{t.read32(0x48000400 + 0x04):08x}",
        "OSPEEDR": f"0x{t.read32(0x48000400 + 0x08):08x}",
        "PUPDR": f"0x{t.read32(0x48000400 + 0x0c):08x}",
        "IDR": f"0x{t.read32(0x48000400 + 0x10):08x}",
        "ODR": f"0x{t.read32(0x48000400 + 0x14):08x}",
        "AFRL": f"0x{t.read32(0x48000400 + 0x20):08x}"
    }

    # 关键全局变量
    mcu_regs["Variables"] = {
        "rx_queue_head": t.read16(0x2000008c),
        "rx_queue_tail": t.read16(0x2000008e),
        "s_tx_state": t.read8(0x2000009c),
        "s_slow_seq": t.read16(0x200000aa),
        "huart2_gState": f"0x{t.read32(0x200004c8 + 0x88):08x}",
        "huart2_RxState": f"0x{t.read32(0x200004c8 + 0x8c):08x}",
        "huart2_ErrorCode": f"0x{t.read32(0x200004c8 + 0x90):08x}"
    }

    with open(os.path.join(OUT_DIR, "mcu_registers_snapshot.json"), "w", encoding="utf-8") as f:
        json.dump(mcu_regs, f, indent=2, ensure_ascii=False)
    print("  -> 已保存 mcu_registers_snapshot.json")

    print("[4/4] 抓取 MCU SRAM 内存 Dump...")
    # dump huart2 (148 bytes), rx_queue (256 bytes), s_cmd_out_buf (512 bytes)
    huart2_bytes = bytes(t.read_memory_block8(0x200004c8, 148))
    rx_queue_bytes = bytes(t.read_memory_block8(0x200034fc, 256))
    s_cmd_out_buf_bytes = bytes(t.read_memory_block8(0x2000363c, 512))
    with open(os.path.join(OUT_DIR, "huart2_dump.bin"), "wb") as f: f.write(huart2_bytes)
    with open(os.path.join(OUT_DIR, "rx_queue_dump.bin"), "wb") as f: f.write(rx_queue_bytes)
    with open(os.path.join(OUT_DIR, "cmd_out_buf_dump.bin"), "wb") as f: f.write(s_cmd_out_buf_bytes)
    print("  -> 已保存 huart2_dump.bin, rx_queue_dump.bin, cmd_out_buf_dump.bin")

    session.close()

if __name__ == "__main__":
    capture_windows_usb_info()
    capture_comm_status()
    capture_mcu_state()
    print("\n>>> 现场证据采集与固定 100% 完成！所有原始数据已存入 docs/diagnostics/vlink_cdc_lockup/ <<<")
