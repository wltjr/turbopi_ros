#!/usr/bin/env python3
"""
sniff_serial.py – dump raw bytes from STM32 board serial port.

Run on the Pi (with ros2_control_node NOT running):
    python3 sniff_serial.py [device] [baudrate]

Examples:
    python3 sniff_serial.py /dev/ttyAMA0 1000000
    python3 sniff_serial.py /dev/rrc 1000000
"""
import serial
import sys
import time
import subprocess

DURATION = 10  # seconds to listen

def try_device(device, baudrate):
    print(f"\n{'='*60}")
    print(f"Trying {device} @ {baudrate} baud for {DURATION}s ...")
    try:
        port = serial.Serial(device, baudrate, timeout=1)
        port.rts = False
        port.dtr = False
    except Exception as e:
        print(f"  Cannot open: {e}")
        return 0

    buf = bytearray()
    start = time.time()
    while time.time() - start < DURATION:
        chunk = port.read(256)
        if chunk:
            buf.extend(chunk)
    port.close()

    total = len(buf)
    print(f"  Total bytes received: {total}")

    if total == 0:
        print("  *** NO DATA ***")
        return 0

    # Hex dump
    print(f"  First {min(64, total)} bytes:")
    for i in range(0, min(64, total), 16):
        row = buf[i:i+16]
        hex_str = " ".join(f"{b:02X}" for b in row)
        asc_str = "".join(chr(b) if 32 <= b < 127 else '.' for b in row)
        print(f"    {i:04d}: {hex_str:<48}  {asc_str}")

    # Scan for AA 55 frames
    count = 0
    i = 0
    while i < len(buf) - 1:
        if buf[i] == 0xAA and buf[i+1] == 0x55:
            end = min(i + 12, len(buf))
            hex_str = " ".join(f"{b:02X}" for b in buf[i:end])
            print(f"  Frame @ offset {i}: {hex_str}")
            count += 1
            i += 2
        else:
            i += 1
    print(f"  AA 55 frames found: {count}")
    return total

def main():
    # Show available devices
    print("Available serial devices:")
    r = subprocess.run("ls -la /dev/ttyAMA* /dev/ttyACM* /dev/ttyUSB* /dev/rrc 2>/dev/null",
                       shell=True, capture_output=True, text=True)
    print(r.stdout or "  (none found)")

    if len(sys.argv) >= 3:
        # User specified device and baudrate
        try_device(sys.argv[1], int(sys.argv[2]))
        return

    if len(sys.argv) == 2:
        try_device(sys.argv[1], 1000000)
        return

    # Auto-probe: try ttyAMA0, ttyAMA10, rrc at 1Mbaud
    devices = ["/dev/ttyAMA0", "/dev/ttyAMA10", "/dev/rrc"]
    for dev in devices:
        r = subprocess.run(f"test -e {dev}", shell=True)
        if r.returncode == 0:
            result = try_device(dev, 1000000)
            if result > 0:
                print(f"\n✓ Found data on {dev} @ 1000000 baud")
                break
    else:
        print("\nNo data found on any device at 1Mbaud.")
        print("Try manually: python3 sniff_serial.py /dev/ttyAMA0 1000000")

if __name__ == "__main__":
    main()
