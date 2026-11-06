#!/usr/bin/env python3
"""Capture the RP2350 RX debug log (USB CDC, VID 0x2e8a) for N seconds. Never opens at 1200 baud."""
import sys, time
import serial, serial.tools.list_ports

def main():
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 30
    out = open(sys.argv[2], 'a') if len(sys.argv) > 2 else sys.stdout
    port = next((p.device for p in serial.tools.list_ports.comports() if p.vid == 0x2E8A), None)
    if not port:
        sys.exit('no RP2 port found')
    s = serial.Serial(port, 420000, timeout=0.5)
    end = time.time() + seconds
    while time.time() < end:
        line = s.readline().decode(errors='replace').rstrip()
        if line:
            out.write(f'{time.strftime("%H:%M:%S")} {line}\n')
            out.flush()
    s.close()

if __name__ == '__main__':
    main()
