#!/usr/bin/env python3
import time

import serial

ser = serial.Serial("/dev/AMAl0", baudrate=115200, timeout=1)

print("UART Echo test 시작 (Ctrl+C 종료)")
try:
    cnt = 0
    while True:
        msg = f"Hello from Raspberry Pi {cnt}\n"
        ser.write(msg.encode())
        print("TX:", msg.strip())
        cnt += 1

        line = ser.readline().decode(errors="ignore").strip()
        if line:
            print("RX:", line)

        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    ser.close()
