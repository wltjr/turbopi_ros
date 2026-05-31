#!/usr/bin/env python3
"""
test_board.py – test STM32 board communication using the official Pi5 Python SDK.

Run on the Pi (with ros2_control_node NOT running, so /dev/rrc is free):
    python3 test_board.py

This script:
1. Opens the board
2. Sends a stop-motors command (to wake up the STM32)
3. Listens for 10s and reports any battery/IMU data received
"""
import sys
import time
import struct

# Try to import from the Pi5 SDK location
sdk_paths = [
    '/home/marce/ros2_ws/src/turbopi_ros2_pi5/src/driver/sdk/sdk',
    '/home/ubuntu/ros2_ws/src/turbopi_ros2_pi5/src/driver/sdk/sdk',
]
for p in sdk_paths:
    sys.path.insert(0, p)

try:
    from ros_robot_controller_sdk import Board
    print("Imported Board from Pi5 SDK")
except ImportError:
    # Fall back to inline implementation for testing
    print("Pi5 SDK not found, using inline implementation")
    import serial
    import threading
    import queue

    crc8_table = [
        0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
        157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
        35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
        190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
        70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
        219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
        101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
        248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
        140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
        17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
        175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
        50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
        202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
        87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
        233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
        116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53
    ]

    def checksum_crc8(data):
        check = 0
        for b in data:
            check = crc8_table[check ^ b]
        return check & 0xFF

    class Board:
        def __init__(self, device="/dev/rrc", baudrate=1000000):
            self.port = serial.Serial(None, baudrate, timeout=1)
            self.port.rts = False
            self.port.dtr = False
            self.port.setPort(device)
            self.port.open()
            self.enable_recv = False
            self.frame = []
            self.recv_count = 0
            self.raw_bytes = bytearray()
            self.sys_queue = queue.Queue(maxsize=10)
            self.imu_queue = queue.Queue(maxsize=10)
            threading.Thread(target=self._recv, daemon=True).start()

        def buf_write(self, func, data):
            buf = [0xAA, 0x55, int(func)]
            buf.append(len(data))
            buf.extend(data)
            buf.append(checksum_crc8(bytes(buf[2:])))
            self.port.write(buf)

        def set_motor_speed(self, speeds):
            data = [0x01, len(speeds)]
            for i in speeds:
                data.extend(struct.pack("<Bf", int(i[0] - 1), float(i[1])))
            self.buf_write(3, data)  # PACKET_FUNC_MOTOR = 3

        def enable_reception(self, v=True):
            self.enable_recv = v

        def get_battery(self):
            try:
                data = self.sys_queue.get(block=False)
                if data[0] == 0x04:
                    return struct.unpack('<H', data[1:3])[0]
            except queue.Empty:
                pass
            return None

        def _recv(self):
            STATE_START1, STATE_START2, STATE_FUNC, STATE_LEN, STATE_DATA, STATE_CRC = range(6)
            state = STATE_START1
            frame = []
            length = 0
            count = 0
            while True:
                if not self.enable_recv:
                    time.sleep(0.01)
                    continue
                b = self.port.read(1)
                if not b:
                    continue
                dat = b[0]
                self.raw_bytes.append(dat)

                if state == STATE_START1:
                    if dat == 0xAA:
                        state = STATE_START2
                elif state == STATE_START2:
                    state = STATE_FUNC if dat == 0x55 else STATE_START1
                elif state == STATE_FUNC:
                    if dat < 12:
                        frame = [dat, 0]
                        state = STATE_LEN
                    else:
                        state = STATE_START1
                elif state == STATE_LEN:
                    frame[1] = dat
                    length = dat
                    count = 0
                    state = STATE_CRC if dat == 0 else STATE_DATA
                elif state == STATE_DATA:
                    frame.append(dat)
                    count += 1
                    if count >= length:
                        state = STATE_CRC
                elif state == STATE_CRC:
                    expected = checksum_crc8(bytes(frame))
                    if expected == dat:
                        func = frame[0]
                        payload = bytes(frame[2:])
                        if func == 0:  # SYS
                            self.sys_queue.put_nowait(payload)
                        elif func == 7:  # IMU
                            self.imu_queue.put_nowait(payload)
                        print(f"  [RX] func={func:#04x} len={frame[1]} payload={payload.hex()}")
                    else:
                        print(f"  [RX] CRC mismatch: got {dat:#04x} expected {expected:#04x}")
                    state = STATE_START1


def main():
    print(f"Opening /dev/rrc ...")
    try:
        board = Board()
    except Exception as e:
        print(f"Failed: {e}")
        return

    board.enable_reception(True)
    print("Board opened. Sending stop-motors command to wake up STM32...")
    board.set_motor_speed([[1, 0], [2, 0], [3, 0], [4, 0]])

    print("Listening for 15s — any SYS/IMU packets will be printed above...")
    for i in range(15):
        time.sleep(1)
        v = board.get_battery()
        if v is not None:
            print(f"\n✓ Battery: {v} mV = {v/1000:.2f} V")
        print(f"  {i+1}s elapsed, raw bytes received: {len(board.raw_bytes)}")

    print(f"\nTotal raw bytes received in 15s: {len(board.raw_bytes)}")
    if len(board.raw_bytes) > 0:
        print("First 64 bytes (hex):")
        chunk = board.raw_bytes[:64]
        for i in range(0, len(chunk), 16):
            row = chunk[i:i+16]
            print(f"  {' '.join(f'{b:02X}' for b in row)}")
    else:
        print("NO DATA RECEIVED from STM32 board.")
        print("Check physical connections on GPIO header.")

if __name__ == "__main__":
    main()
