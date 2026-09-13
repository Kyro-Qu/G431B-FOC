# -*- coding: utf-8 -*-
"""pyocd halt 抓现场：PC / uwTick / USART2 & DMA 寄存器 / 遥测 TX 状态机 (ASCII only)"""
import sys
import time
import struct
from pyocd.core.helpers import ConnectHelper

HUART2 = 0x20000490
UWTICK = 0x20000018
S_TX_STATE = 0x20000094       # 1B: 0 idle, 1 wave, 2 slow
TELEM_ENABLE = 0x20000095
TELEM_SUSPEND = 0x20000096
S_STATUS_PENDING = 0x20000097
S_EVENT_PENDING = 0x20000098
S_ACK_PENDING = 0x20000099
S_WAVE_SEQ = 0x2000009C
S_SLOW_SEQ = 0x2000009E
S_CHANNEL_MASK = 0x200000A0
S_LAST_STATUS_MS = 0x200000A4
RX_QUEUE_HEAD = 0x20000088
RX_QUEUE_TAIL = 0x2000008A
RX_LAST_POS = 0x2000008C
G_CPU_DIAG = 0x20006AC0

USART2 = 0x40004400
DMA1 = 0x40020000


def dump(t, tag):
    pc = t.read_core_register('pc')
    lr = t.read_core_register('lr')
    sp = t.read_core_register('sp')
    xpsr = t.read_core_register('xpsr')
    print('[%s] PC=0x%08x LR=0x%08x SP=0x%08x IPSR=%d' % (tag, pc, lr, sp, xpsr & 0x1FF))
    print('  uwTick=%u' % t.read32(UWTICK))
    print('  huart2: Lock=%u gState=0x%02x RxState=0x%02x ErrorCode=0x%08x TxXferCount=%u RxXferSize=%u ReceptionType=%u'
          % (t.read8(HUART2 + 132), t.read32(HUART2 + 136), t.read32(HUART2 + 140),
             t.read32(HUART2 + 144), t.read16(HUART2 + 86), t.read16(HUART2 + 92), t.read32(HUART2 + 108)))
    print('  telem: tx_state=%u enable=%u suspend=%u pend(st/ev/ack)=%u/%u/%u wave_seq=%u slow_seq=%u mask=0x%08x last_status_ms=%u'
          % (t.read8(S_TX_STATE), t.read8(TELEM_ENABLE), t.read8(TELEM_SUSPEND),
             t.read8(S_STATUS_PENDING), t.read8(S_EVENT_PENDING), t.read8(S_ACK_PENDING),
             t.read16(S_WAVE_SEQ), t.read16(S_SLOW_SEQ), t.read32(S_CHANNEL_MASK), t.read32(S_LAST_STATUS_MS)))
    print('  cli rx: head=%u tail=%u last_pos=%u' % (t.read16(RX_QUEUE_HEAD), t.read16(RX_QUEUE_TAIL), t.read16(RX_LAST_POS)))
    print('  cpu: last=%u max=%u load=%.1f%%' % (t.read32(G_CPU_DIAG), t.read32(G_CPU_DIAG + 4),
                                                 struct.unpack('<f', struct.pack('<I', t.read32(G_CPU_DIAG + 8)))[0]))
    cr1 = t.read32(USART2 + 0x00)
    cr3 = t.read32(USART2 + 0x08)
    isr = t.read32(USART2 + 0x1C)
    print('  USART2: CR1=0x%08x (UE=%u RE=%u TE=%u IDLEIE=%u TCIE=%u) CR3=0x%08x (DMAR=%u DMAT=%u EIE=%u) BRR=%u ISR=0x%08x (TXE=%u TC=%u RXNE=%u ORE=%u IDLE=%u)'
          % (cr1, cr1 & 1, (cr1 >> 2) & 1, (cr1 >> 3) & 1, (cr1 >> 4) & 1, (cr1 >> 6) & 1,
             cr3, (cr3 >> 6) & 1, (cr3 >> 7) & 1, cr3 & 1, t.read32(USART2 + 0x0C), isr,
             (isr >> 7) & 1, (isr >> 6) & 1, (isr >> 5) & 1, (isr >> 3) & 1, (isr >> 4) & 1))
    print('  DMA1 ISR=0x%08x' % t.read32(DMA1 + 0x00))
    print('  DMA1_Ch1(TX): CCR=0x%08x CNDTR=%u CPAR=0x%08x CMAR=0x%08x'
          % (t.read32(DMA1 + 0x08), t.read32(DMA1 + 0x0C), t.read32(DMA1 + 0x10), t.read32(DMA1 + 0x14)))
    print('  DMA1_Ch2(RX): CCR=0x%08x CNDTR=%u CPAR=0x%08x CMAR=0x%08x'
          % (t.read32(DMA1 + 0x1C), t.read32(DMA1 + 0x20), t.read32(DMA1 + 0x24), t.read32(DMA1 + 0x28)))
    icsr = t.read32(0xE000ED04)
    print('  NVIC ICSR=0x%08x VECTACTIVE=%u VECTPENDING=%u ISPR0=0x%08x ISER0=0x%08x ISER1=0x%08x'
          % (icsr, icsr & 0x1FF, (icsr >> 12) & 0x1FF, t.read32(0xE000E200), t.read32(0xE000E100), t.read32(0xE000E104)))
    print('  CFSR=0x%08x HFSR=0x%08x' % (t.read32(0xE000ED28), t.read32(0xE000ED2C)))


with ConnectHelper.session_with_chosen_probe(options={"frequency": 4000000, "connect_mode": "attach"}) as session:
    t = session.target
    t.halt()
    dump(t, 'halt#1')
    t.resume()
    time.sleep(0.5)
    t.halt()
    dump(t, 'halt#2 (+0.5s)')
    t.resume()
