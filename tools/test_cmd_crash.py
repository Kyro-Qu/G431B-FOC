# -*- coding: utf-8 -*-
"""
Test script to check if 'feedback' and 'sensorless status' commands cause MCU crash/reset.
"""

import sys
import time
import re
import serial

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

PORT = "COM44"
BAUD = 6500000


def open_serial():
    print(f"[*] Connecting to {PORT} at {BAUD} baud...")
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    return ser


def send_cmd(ser, cmd_str, wait_time=0.35):
    print(f"\n==========================================")
    print(f">>> TX: {cmd_str}")
    ser.reset_input_buffer()
    ser.write((cmd_str + "\r\n").encode("utf-8"))

    t0 = time.time()
    buf = bytearray()
    while time.time() - t0 < wait_time:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
        else:
            time.sleep(0.01)

    text = buf.decode("utf-8", errors="replace")
    print(f"<<< RX ({len(buf)} bytes):")
    if text.strip():
        print(text.strip())
    else:
        print("[NO RESPONSE / EMPTY]")
    print(f"==========================================")
    return text


def parse_status_fields(status_text):
    calib_match = re.search(r"calib=(\d+)(\*?)", status_text)
    rst_flags_match = re.search(r"rst_flags=(0x[0-9a-fA-F]+)", status_text)
    chk_match = re.search(r"chk=(\d+)", status_text)
    fault_match = re.search(r"fault=(\d+)", status_text)

    return {
        "calib": calib_match.group(0) if calib_match else None,
        "rst_flags": rst_flags_match.group(1) if rst_flags_match else None,
        "chk": chk_match.group(1) if chk_match else None,
        "fault": fault_match.group(1) if fault_match else None,
    }


def main():
    try:
        ser = open_serial()
    except Exception as e:
        print(f"[!] Failed to open serial port {PORT}: {e}")
        sys.exit(1)

    try:
        # Step 2: Send 'fault clear'
        print("\n--- STEP 2: Send 'fault clear' ---")
        rx_fc = send_cmd(ser, "fault clear", wait_time=0.25)

        # Step 3: Send 'status' -> print status
        print("\n--- STEP 3: Send 'status' ---")
        rx_status1 = send_cmd(ser, "status", wait_time=0.4)
        info1 = parse_status_fields(rx_status1)
        print(f"Parsed initial status fields: {info1}")

        # Step 4: Send 'feedback' -> print response. Did it reply? Did MCU reset?
        print("\n--- STEP 4: Send 'feedback' ---")
        rx_feedback = send_cmd(ser, "feedback", wait_time=0.4)
        has_feedback_reply = "feedback:" in rx_feedback
        reset_indicators_in_fb = "config loaded" in rx_feedback or "firmware=" in rx_feedback
        print(f"[*] 'feedback' evaluation:")
        print(f"    - Replied with expected header: {has_feedback_reply}")
        print(f"    - Reboot banner detected in response: {reset_indicators_in_fb}")

        # Step 5: Send 'sensorless status' -> print response. Did it reply? Did MCU reset?
        print("\n--- STEP 5: Send 'sensorless status' ---")
        rx_sl = send_cmd(ser, "sensorless status", wait_time=0.4)
        has_sl_reply = "sensorless:" in rx_sl
        reset_indicators_in_sl = "config loaded" in rx_sl or "firmware=" in rx_sl
        print(f"[*] 'sensorless status' evaluation:")
        print(f"    - Replied with expected header: {has_sl_reply}")
        print(f"    - Reboot banner detected in response: {reset_indicators_in_sl}")

        # Step 6: Check 'status' again, see if rst_flags or calib changed.
        print("\n--- STEP 6: Send 'status' again ---")
        rx_status2 = send_cmd(ser, "status", wait_time=0.4)
        info2 = parse_status_fields(rx_status2)
        print(f"Parsed second status fields: {info2}")

        print("\n" + "=" * 50)
        print("SUMMARY REPORT:")
        print("=" * 50)
        print(f"Status 1 fields: {info1}")
        print(f"Status 2 fields: {info2}")

        calib_changed = (info1["calib"] != info2["calib"])
        rst_flags_changed = (info1["rst_flags"] != info2["rst_flags"])
        chk_changed = (info1["chk"] != info2["chk"])

        print(f"calib changed: {calib_changed} ('{info1.get('calib')}' -> '{info2.get('calib')}')")
        print(f"rst_flags changed: {rst_flags_changed} ('{info1.get('rst_flags')}' -> '{info2.get('rst_flags')}')")
        print(f"chk changed: {chk_changed} ('{info1.get('chk')}' -> '{info2.get('chk')}')")

        if not has_feedback_reply:
            print("[CRITICAL] 'feedback' command did NOT reply correctly!")
        else:
            print("[OK] 'feedback' command replied successfully.")

        if not has_sl_reply:
            print("[CRITICAL] 'sensorless status' command did NOT reply correctly!")
        else:
            print("[OK] 'sensorless status' command replied successfully.")

        if reset_indicators_in_fb or reset_indicators_in_sl:
            print("[CRITICAL] MCU reset detected during command execution!")
        elif calib_changed or rst_flags_changed:
            print("[WARNING] Status variables changed unexpectedly, inspect closely.")
        else:
            print("[OK] MCU did NOT reset. No crash detected.")

    finally:
        ser.close()
        print("\n[*] Serial port closed.")


if __name__ == "__main__":
    main()
