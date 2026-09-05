# -*- coding: utf-8 -*-
"""SWD 深度诊断: uwTick 是否前进 / huart2 状态 / USART2 外设寄存器"""
import sys
import time

from pyocd.core.helpers import ConnectHelper
from pyocd.core.memory_map import MemoryType

sess = ConnectHelper.session_with_chosen_probe(
    unique_id="Vllink.Basic2.502DFFAE76",
    target_override="cortex_m",
    options={"frequency": 1000000})
board = sess.board
target = board.target
core = None
for k, v in target.cores.items():
    print("core key:", k, v)
    core = v
if core is None:
    # cortex_m 通用目标: 直接用目标级 API 前先选核
    raise SystemExit("no cores enumerated")

T().halt()
print("== halted ==")

# PRIMASK / 控制寄存器
try:
    pm = T().read_core_register("primask")
    print("PRIMASK = 0x%x" % pm)
except Exception as e:
    print("primask read fail:", e)

# uwTick 两次采样: halt 读 -> resume 1s -> halt 再读
uwtick_addr = 0x20000018
t1 = T().read32(uwtick_addr)
print("uwTick t1 = %d" % t1)
T().resume()
time.sleep(1.0)
T().halt()
t2 = T().read32(uwtick_addr)
print("uwTick t2 = %d  (delta=%d)" % (t2, t2 - t1))
print("PC = 0x%08x  LR = 0x%08x" % (T().read_core_register("pc"),
                                    T().read_core_register("lr")))

# huart2 完整 dump (148 B = 37 words)
print("== huart2 @0x20000460 ==")
words = T().read32(0x20000460, 36)
for i in range(0, 36, 4):
    print("  +0x%02x: %s" % (i * 4, " ".join("%08x" % w for w in words[i:i + 4])))

# USART2 外设: CR1 CR2 CR3 BRR GTPR RTOR RQR ISR ICR RDR TDR
print("== USART2 @0x40004400 ==")
regs = [("CR1", 0x00), ("CR2", 0x04), ("CR3", 0x08), ("BRR", 0x0C),
        ("GTPR", 0x10), ("RTOR", 0x14), ("RQR", 0x18), ("ISR", 0x1C),
        ("ICR", 0x20), ("RDR", 0x24), ("TDR", 0x28), ("PRESC", 0x2C)]
for name, off in regs:
    v = T().read32(0x40004400 + off, 1)
    print("  %-6s = 0x%08x" % (name, v[0]))

# RCC: APB1ENR1 bit17 USART2EN, APB1RSTR1
print("== RCC ==")
for name, addr in [("APB1ENR1", 0x40021058), ("APB1RSTR1", 0x40021020),
                   ("AHB2ENR", 0x4002104C)]:
    v = T().read32(addr, 1)
    print("  %-10s = 0x%08x" % (name, v[0]))

# GPIOA/C AF 配置: USART2_TX=PA2 AF7, RX=PA3 AF7
print("== GPIOA (PA2/PA3) ==")
for name, addr in [("MODER", 0x48000000), ("AFRL", 0x48000020),
                   ("OSPEEDR", 0x48000008)]:
    v = T().read32(addr, 1)
    print("  %-8s = 0x%08x" % (name, v[0]))

T().resume()
print("== resumed, exiting ==")
sess.close()
