# -*- coding: utf-8 -*-
import time
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
if t.is_halted(): t.resume()

def send_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'): cmd_str += '\r\n'
    raw = cmd_str.encode('ascii')
    head = t.read16(0x2000008c)
    for b in raw:
        t.write8(0x20003508 + head, b)
        head = (head + 1) % 256
    t.write16(0x2000008c, head)
    if t.is_halted(): t.resume()
    t0 = time.time()
    while time.time() - t0 < 0.5:
        if t.read16(0x2000008e) == head: break
        time.sleep(0.01)

send_cmd("disable")
time.sleep(0.05)
data = bytes(t.read_memory_block8(0x20002e1c, 0x140))
state = data[0x9e]
mode = data[0x9f]
print(f"停机完成: state={state} (0=IDLE), mode={mode}")
session.close()
