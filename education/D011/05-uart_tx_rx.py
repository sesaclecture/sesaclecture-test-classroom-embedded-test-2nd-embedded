#!/usr/bin/env python3
import time

import serial

dev = "/dev/ttyAMA2"

ser = serial.Serial(dev, baudrate=115200, timeout=1)

try:
    for i in range(5):
        msg = f"Hello World {i}\n"
        ser.write(msg.encode())
        print("TX:", msg.strip())

        line = ser.readline().decode().strip()
        print("RX:", line)

        time.sleep(1)
finally:
    ser.close()
