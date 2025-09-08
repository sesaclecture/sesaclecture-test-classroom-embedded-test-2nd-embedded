#!/usr/bin/env python3
# Raspberry Pi + MPU6050 (I2C 0x68)
# WHO_AM_I 확인 → 초기화(PWR_MGMT_1=0) → RAW accel/gyro 값 출력

import time

from smbus2 import SMBus

I2C_BUS   = 1
MPU_ADDR  = 0x68

REG_WHOAMI = 0x75
REG_PWR1   = 0x6B
REG_ACCEL  = 0x3B   # ACCEL_XOUT_H
REG_GYRO   = 0x43   # GYRO_XOUT_H

def twos_complement_16(msb: int, lsb: int) -> int:
    """두 바이트를 16비트 signed int로 변환"""
    val = (msb << 8) | lsb
    if val & 0x8000:
        val -= 0x10000
    return val

def init_mpu():
    with SMBus(I2C_BUS) as bus:
        who = bus.read_byte_data(MPU_ADDR, REG_WHOAMI)
        if who != 0x68:
            raise RuntimeError(f"WHO_AM_I mismatch: 0x{who:02X}")
        bus.write_byte_data(MPU_ADDR, REG_PWR1, 0x00)  # 슬립 해제
        time.sleep(0.05)

def read_raw_accel_gyro():
    with SMBus(I2C_BUS) as bus:
        acc = bus.read_i2c_block_data(MPU_ADDR, REG_ACCEL, 6)
        gyr = bus.read_i2c_block_data(MPU_ADDR, REG_GYRO, 6)

    ax = twos_complement_16(acc[0], acc[1])
    ay = twos_complement_16(acc[2], acc[3])
    az = twos_complement_16(acc[4], acc[5])

    gx = twos_complement_16(gyr[0], gyr[1])
    gy = twos_complement_16(gyr[2], gyr[3])
    gz = twos_complement_16(gyr[4], gyr[5])

    return (ax, ay, az), (gx, gy, gz)

if __name__ == "__main__":
    init_mpu()
    print("MPU6050 RAW data output. Ctrl+C to stop.")

    try:
        while True:
            (ax, ay, az), (gx, gy, gz) = read_raw_accel_gyro()
            print(f"ACC: {ax:6d} {ay:6d} {az:6d} | GYRO: {gx:6d} {gy:6d} {gz:6d}")
            time.sleep(0.05)  # 대략 20Hz 출력
    except KeyboardInterrupt:
        pass
