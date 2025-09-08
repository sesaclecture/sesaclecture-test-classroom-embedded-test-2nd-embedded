#!/usr/bin/env python3
import struct
import time
from collections import deque

import matplotlib.pyplot as plt
from smbus2 import SMBus

I2C_BUS    = 1
MPU_ADDR   = 0x68
REG_WHOAMI = 0x75
REG_PWR1   = 0x6B
REG_ACCEL  = 0x3B  # ACCEL_XOUT_H
REG_GYRO   = 0x43  # GYRO_XOUT_H

# 스케일링(기본 레인지 가정)
ACC_LSB_PER_G   = 16384.0
GYRO_LSB_PER_DPS= 131.0

def read_i2c_block(bus, addr, reg, n):
    return bus.read_i2c_block_data(addr, reg, n)

def i16(msb, lsb):
    val = (msb << 8) | lsb
    return struct.unpack(">h", bytes([msb, lsb]))[0]  # signed 16-bit

def init_mpu():
    with SMBus(I2C_BUS) as bus:
        # WHO_AM_I 확인
        who = bus.read_byte_data(MPU_ADDR, REG_WHOAMI)
        if who != 0x68:
            raise RuntimeError(f"WHO_AM_I mismatch: 0x{who:02X}")
        # 슬립 해제
        bus.write_byte_data(MPU_ADDR, REG_PWR1, 0x00)
        time.sleep(0.05)

def read_accel_gyro():
    with SMBus(I2C_BUS) as bus:
        # 6바이트씩 읽기
        acc = read_i2c_block(bus, MPU_ADDR, REG_ACCEL, 6)
        gyr = read_i2c_block(bus, MPU_ADDR, REG_GYRO, 6)

    ax = i16(acc[0], acc[1]) / ACC_LSB_PER_G
    ay = i16(acc[2], acc[3]) / ACC_LSB_PER_G
    az = i16(acc[4], acc[5]) / ACC_LSB_PER_G

    gx = i16(gyr[0], gyr[1]) / GYRO_LSB_PER_DPS
    gy = i16(gyr[2], gyr[3]) / GYRO_LSB_PER_DPS
    gz = i16(gyr[4], gyr[5]) / GYRO_LSB_PER_DPS
    return (ax, ay, az), (gx, gy, gz)

def main():
    init_mpu()
    print("MPU6050 live read start: Ctrl+C to stop")
    print(" ax(g)   ay(g)   az(g)  |  gx(dps)  gy(dps)  gz(dps)")

    # 최근 N개를 그래프에 표시(슬라이딩 윈도우)
    N = 100
    buf_ax = deque(maxlen=N); buf_ay = deque(maxlen=N); buf_az = deque(maxlen=N)
    buf_gx = deque(maxlen=N); buf_gy = deque(maxlen=N); buf_gz = deque(maxlen=N)

    # Matplotlib 준비 (서브플롯 없이 두 번 그리면 느려지니, 단일 fig에 두 개 라인 그룹)
    plt.ion()
    fig, ax1 = plt.subplots()
    ax1.set_title("MPU6050 Live (Accel g / Gyro dps)")
    ax1.set_xlabel("samples")
    l_ax, = ax1.plot([], [], label="ax (g)")
    l_ay, = ax1.plot([], [], label="ay (g)")
    l_az, = ax1.plot([], [], label="az (g)")
    ax1.legend(loc="upper left")

    ax2 = ax1.twinx()
    l_gx, = ax2.plot([], [], linestyle="--", label="gx (dps)")
    l_gy, = ax2.plot([], [], linestyle="--", label="gy (dps)")
    l_gz, = ax2.plot([], [], linestyle="--", label="gz (dps)")
    ax2.legend(loc="upper right")

    try:
        while True:
            (ax, ay, az), (gx, gy, gz) = read_accel_gyro()

            # 터미널 출력
            print(f"{ax:+.3f} {ay:+.3f} {az:+.3f} | {gx:+.1f} {gy:+.1f} {gz:+.1f}")

            # 버퍼 업데이트
            buf_ax.append(ax); buf_ay.append(ay); buf_az.append(az)
            buf_gx.append(gx); buf_gy.append(gy); buf_gz.append(gz)

            # 그래프 업데이트(간단 실시간)
            xs = range(len(buf_ax))
            l_ax.set_data(xs, list(buf_ax))
            l_ay.set_data(xs, list(buf_ay))
            l_az.set_data(xs, list(buf_az))
            l_gx.set_data(xs, list(buf_gx))
            l_gy.set_data(xs, list(buf_gy))
            l_gz.set_data(xs, list(buf_gz))

            ax1.relim(); ax1.autoscale_view()
            ax2.relim(); ax2.autoscale_view()
            plt.pause(0.01)  # non-blocking 업데이트

            time.sleep(0.01)  # 센서 폴링 속도(약 100Hz 근사)

    except KeyboardInterrupt:
        pass
    finally:
        plt.ioff()
        plt.show()

if __name__ == "__main__":
    main()
