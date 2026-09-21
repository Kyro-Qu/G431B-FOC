# -*- coding: utf-8 -*-
import time
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
if t.is_halted(): t.resume()

head = t.read16(0x2000008c)
tail = t.read16(0x2000008e)
print(f"初始状态: head={head}, tail={tail}, is_halted={t.is_halted()}")

# 写入 "fault clear\r\n"
cmd = b"fault clear\r\n"
for b in cmd:
    t.write8(0x200034fc + head, b)
    head = (head + 1) % 256
t.write16(0x2000008c, head)

t0 = time.time()
while time.time() - t0 < 1.0:
    cur_tail = t.read16(0x2000008e)
    if cur_tail == head:
        print(f"命令被成功消费! cur_tail={cur_tail}")
        break
    time.sleep(0.02)
else:
    print(f"超时未消费! head={head}, tail={t.read16(0x2000008e)}")

time.sleep(0.05)
raw = bytes(t.read_memory_block8(0x20002e1c + 156, 8))
slow_div = raw[0] | (raw[1] << 8)
state = raw[2]
mode = raw[3]
print(f"执行后状态: slow_div={slow_div}, state={state} (0=IDLE, 1=RUN, 3=FAULT), mode={mode}")

session.close()
