# -*- coding: utf-8 -*-
import sys
from pyocd.core.helpers import ConnectHelper

try:
    session = ConnectHelper.session_with_chosen_probe(target_override="cortex_m")
    if session is None:
        print("PROBE_NONE: 未找到 DAPLink 探针")
        sys.exit(1)
    with session:
        target = session.target
        state = target.get_state()
        print(f"SWD_CONNECTED! CPU state: {state}")
        pc = target.read_core_register("pc")
        sp = target.read_core_register("sp")
        print(f"Registers: PC=0x{pc:08X}, SP=0x{sp:08X}")
except Exception as e:
    print(f"SWD_ERROR: {e}")
