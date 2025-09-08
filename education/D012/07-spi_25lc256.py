import time

import spidev

# SPI 초기화
spi = spidev.SpiDev()
spi.open(0, 0)              # bus=0, device=0 (/dev/spidev0.0)
spi.max_speed_hz = 1_000_000
spi.mode = 0                # 25LC256은 mode0/mode3 모두 OK

# EEPROM 명령어
WREN  = 0x06  # Write Enable
WRITE = 0x02  # Write Data
READ  = 0x03  # Read Data
RDSR  = 0x05  # Read Status Register

def eeprom_write(addr, data: bytes):
    # Write Enable
    spi.xfer2([WREN])
    time.sleep(0.01)

    # Write Data: [WRITE, addr_high, addr_low, data...]
    ah, al = (addr >> 8) & 0xFF, addr & 0xFF
    spi.xfer2([WRITE, ah, al] + list(data))

    # Write 완료 대기 (RDSR의 bit0=Write-In-Progress)
    while True:
        status = spi.xfer2([RDSR, 0])[1]
        if (status & 0x01) == 0:
            break
        time.sleep(0.001)

def eeprom_read(addr, length):
    ah, al = (addr >> 8) & 0xFF, addr & 0xFF
    resp = spi.xfer2([READ, ah, al] + [0x00]*length)
    return bytes(resp[3:])  # 첫 3바이트는 커맨드/주소, 그 뒤가 데이터

# 테스트
test_data = b"HELLO PI"
eeprom_write(0x0000, test_data)
print("READ:", eeprom_read(0x0000, len(test_data)))
