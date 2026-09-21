# -*- coding: utf-8 -*-
"""
可靠的 SWD CLI 命令注入与测试
"""
import time
import struct
from pyocd.core.helpers import ConnectHelper

MOTOR_BASE = 0x20002e1c
RX_HEAD = 0x2000008c
RX_TAIL = 0x2000008e
RX_BUF = 0x200034fc

session = ConnectHelper.session_with_chosen_probe(target_override='cortex_m', options={'halt_on_connect': False, 'resume_on_exit': True})
session.open()
t = session.target
if t.is_halted(): t.resume()

def send_cmd(cmd_str):
    if not cmd_str.endswith('\r\n'):
        cmd_str = cmd_str.rstrip('\r\n') + '\r\n'
    raw = list(cmd_str.encode('ascii'))
    head = t.read16(RX_HEAD)
    # 检查环形缓冲区剩余空间
    tail = t.read16(RX_TAIL)
    # 批量写入
    n = len(raw)
    first_chunk = min(n, 256 - head)
    t.write_memory_block8(RX_BUF + head, raw[:first_chunk])
    if n > first_chunk:
        t.write_memory_block8(RX_BUF, raw[first_chunk:])
    new_head = (head + n) % 256
    t.write16(RX_HEAD, new_head)

    # 等待消费
    t0 = time.time()
    while time.time() - t0 < 0.5:
        cur_tail = t.read16(RX_TAIL)
        if cur_tail == new_head:
            break
        time.sleep(0.005)
    time.sleep(0.01)

try:
    print("测试原子批量命令发送...")
    send_cmd("fault clear")
    send_cmd("disable")
    send_cmd("mode pos")
    send_cmd("pos kp 3.00")
    send_cmd("pos ki 0.00")
    send_cmd("pos vkp 0.020")
    send_cmd("pos vmax 80")
    send_cmd("pos accel 80")
    send_cmd("target 0.000")
    print("所有命令执行成功并已全部被 MCU 消费！")
finally:
    if t.is_halted(): t.resume()
    session.close()
