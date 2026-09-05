# -*- coding: utf-8 -*-
"""时间线复现：开遥测，obs=0 enable，逐 500ms 打印状态——看快环何时死、CPU 多少"""
import serial, time

ser = serial.Serial('COM44', 6500000, timeout=0.5)

def cmd(c, wait=0.4):
    ser.write((c + '\r\n').encode())
    time.sleep(wait)
    return ser.read(65536).decode('gbk', errors='replace')[:300]

def one_line_status():
    ser.write(b'status\r\n')
    time.sleep(0.45)
    txt = ser.read(65536).decode('gbk', errors='replace')
    get = lambda k: next((l.split('=')[1] for l in txt.split('\r\n') if l.startswith(k)), '?')
    cpu = next((l for l in txt.split('\r\n') if l.startswith('cpu=')), 'cpu=?')
    return (f"{get('M0 ')} | {get('vel=')} | {get('iq=')} | "
            f"cs_fault={get('cs_fault=')} | {cpu}")

# 清故障、校准、使能
print(cmd('disable')); time.sleep(0.3)
print(cmd('fault clear')); time.sleep(0.3)
print(cmd('angle enc'))
print(cmd('calib', wait=5.0))
print(cmd('obs 0'))
print(cmd('mode vel'))
print(cmd('target 2000'))
print('--- enable, timeline every 0.6s ---')
print(cmd('enable', wait=0.3))
for i in range(12):
    print(f'[{(i+1)*0.6:.1f}s]', one_line_status())
    time.sleep(0.2)
ser.close()
print('DONE')
