# -*- coding: utf-8 -*-
import sys
import time
from pyocd.core.helpers import ConnectHelper

session = ConnectHelper.session_with_chosen_probe(target_override="cortex_m")
with session:
    target = session.target
    print("Initial state:", target.get_state())
    target.resume()
    time.sleep(0.2)
    print("State after resume:", target.get_state())
    if target.get_state() != target.State.RUNNING:
        pc = target.read_core_register("pc")
        print(f"Halted at PC=0x{pc:08X}")
