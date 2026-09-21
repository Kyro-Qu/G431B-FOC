# -*- coding: utf-8 -*-
import time
import struct
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
if t.is_halted(): t.resume()

def run_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'): cmd_str += '\r\n'
    raw = cmd_str.encode('ascii')
    head = t.read16(0x2000008c)
    for b in raw:
        t.write8(0x20003508 + head, b)
        head = (head + 1) % 256
    t.write16(0x2000008c, head)
    if t.is_halted(): t.resume()
    t0 = time.time()
    while time.time() - t0 < 0.8:
        if t.read16(0x2000008e) == head: break
        time.sleep(0.01)
    time.sleep(0.02)

print("1. 清障与停机...")
run_cmd("fault clear")
run_cmd("disable")

# 检查当前状态
data = bytes(t.read_memory_block8(0x20002e1c, 0x140))
state = data[0x9e]
mode = data[0x9f]
calib_valid = data[0xd0]
print(f"初始状态: state={state}, mode={mode}, calib_valid={calib_valid}")

print("2. 发起校准 (calib)...")
run_cmd("calib")

t0 = time.time()
while time.time() - t0 < 15.0:
    time.sleep(0.5)
    data = bytes(t.read_memory_block8(0x20002e1c, 0x140))
    st = data[0x9e]
    valid = data[0xd0]
    direction = struct.unpack('<b', data[0xd2:0xd3])[0]
    offset = struct.unpack('<f', data[0xd4:0xd8])[0]
    print(f"校准进度: state={st} (0=IDLE, 1=RUN, 2=CALIB), valid={valid}, dir={direction}, offset={offset:.4f}")
    if valid != 0 and st == 0:
        print(">>> 校准成功完成！")
        break
else:
    print("校准超时！")

session.close()
