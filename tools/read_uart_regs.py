# -*- coding: utf-8 -*-
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override="cortex_m")
with session:
    target = session.target
    print("CPU state:", target.get_state())
    pc = target.read_core_register("pc")
    lr = target.read_core_register("lr")
    print(f"PC=0x{pc:08X}, LR=0x{lr:08X}")
    # Read USART2 CR1 and ISR
    # USART2 base: 0x40004400
    # CR1 offset 0x00, ISR offset 0x1C
    cr1 = target.read32(0x40004400)
    isr = target.read32(0x4000441C)
    print(f"USART2 CR1=0x{cr1:08X}, ISR=0x{isr:08X}")
