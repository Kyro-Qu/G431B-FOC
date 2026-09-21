# -*- coding: utf-8 -*-
import struct
from pyocd.core.helpers import ConnectHelper

with ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True}) as session:
    t = session.target
    if t.is_halted(): t.resume()
    raw = bytes(t.read_memory_block8(0x20002e1c + 400, 300))

print("=== 400 ~ 700 字节 ===")
for off in range(0, 250, 4):
    u = struct.unpack_from('<I', raw, off)[0]
    f = struct.unpack_from('<f', raw, off)[0]
    real_off = 400 + off
    if abs(f) > 0.001 and abs(f) < 1e6:
        print(f"[{real_off}] uint=0x{u:08x}, float={f:10.4f}")
