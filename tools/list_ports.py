import serial.tools.list_ports as lp

ports = list(lp.comports())
print(f"Total ports found: {len(ports)}")
for p in ports:
    print(f"Device: {p.device} | Desc: {p.description} | HWID: {p.hwid}")
