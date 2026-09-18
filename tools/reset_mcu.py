# -*- coding: utf-8 -*-
import sys
import time
from pyocd.core.helpers import ConnectHelper

try:
    session = ConnectHelper.session_with_chosen_probe(target_override="cortex_m")
    with session:
        target = session.target
        state = target.get_state()
        print(f"Current CPU state: {state}")
        print("Resetting and running MCU...")
        target.reset()
        time.sleep(0.5)
        new_state = target.get_state()
        print(f"New CPU state: {new_state}")
except Exception as e:
    print(f"Reset error: {e}")
    sys.exit(1)
