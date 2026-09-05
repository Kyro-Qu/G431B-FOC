# -*- coding: utf-8 -*-
"""pyocd halt and dump g_current_shunt_diag (ASCII only)"""
from pyocd.core.helpers import ConnectHelper
import struct

with ConnectHelper.session_with_chosen_probe(options={"frequency": 4000000}) as session:
    t = session.target
    t.init()
    t.halt()
    pc = t.read_core_register('pc')
    print('PC=0x%08x' % pc)

    raw = t.read_memory_block8(0x20005fb0, 116)
    b = bytes(raw)
    u16 = lambda o: struct.unpack_from('<H', b, o)[0]
    u32 = lambda o: struct.unpack_from('<I', b, o)[0]
    f32 = lambda o: struct.unpack_from('<f', b, o)[0]
    print('--- g_current_shunt_diag ---')
    print('offset u/v/w/wi: %d %d %d %d' % (u16(0), u16(2), u16(4), u16(6)))
    print('adc1_raw=%d adc2_raw=%d' % (u16(8), u16(10)))
    print('current u/v/w: %.3f %.3f %.3f' % (f32(12), f32(16), f32(20)))
    print('sample_count=%d tim_update=%d adc_irq=%d' % (u32(24), u32(28), u32(32)))
    print('rejected_total=%d' % u32(36))
    print('rejected cur u/v/w: %.3f %.3f %.3f' % (f32(40), f32(44), f32(48)))
    print('rejected adc1=%d adc2=%d' % (u16(52), u16(54)))
    print('rejected ccr1-4: %d %d %d %d' % (u16(56), u16(58), u16(60), u16(62)))
    print('rejected_pair=%d rejected_consec=%d' % (b[64], b[65]))
    print('deferred_total=%d deferred_consec=%d' % (u32(68), b[72]))
    print('adc1_isr=0x%08x adc2_isr=0x%08x' % (u32(76), u32(80)))
    print('adc1_jsqr=0x%08x adc2_jsqr=0x%08x' % (u32(84), u32(88)))
    print('adc1_cr=0x%08x adc2_cr=0x%08x' % (u32(92), u32(96)))
    print('tim_cnt=%d tim_cr1=0x%08x' % (u32(100), u32(104)))
    print('state=%d fault_code=%d init_stage=%d' % (b[108], b[109], b[110]))
    print('active_pair=%d pending_pair=%d armed=%d sector=%d'
          % (b[111], b[112], b[113], b[114]))

    print('ICSR=0x%08x CFSR=0x%08x' % (t.read32(0xE000ED04), t.read32(0xE000ED28)))
    print('VECTPENDING=%d' % ((t.read32(0xE000ED04) >> 12) & 0x1FF))
    t.resume()
