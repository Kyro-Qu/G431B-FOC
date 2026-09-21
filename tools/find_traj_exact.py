# -*- coding: utf-8 -*-
import struct
from pyocd.core.helpers import ConnectHelper

with ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True}) as session:
    t = session.target
    if t.is_halted(): t.resume()
    raw = bytes(t.read_memory_block8(0x20002e1c, 824))

print("=== 4 字节对齐扫描 720 ~ 824 字节 ===")
for off in range(720, 821, 4):
    f = struct.unpack_from('<f', raw, off)[0]
    u = struct.unpack_from('<I', raw, off)[0]
    print(f"[{off:03d}] uint=0x{u:08x}, float={f:12.4f}")
