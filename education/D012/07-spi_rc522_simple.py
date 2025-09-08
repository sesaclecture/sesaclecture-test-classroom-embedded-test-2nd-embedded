import spidev

spi = spidev.SpiDev()
spi.open(0, 0)              # bus 0, device 0 (/dev/spidev0.0)
spi.max_speed_hz = 1_000_000
spi.mode = 0

def read_reg(addr):
    # RC522는 어드레스 7비트 <<1 + 0x80(read) + dummy
    val = spi.xfer2([((addr << 1) & 0x7E) | 0x80, 0])
    return val[1]

version = read_reg(0x37)
print("RC522 Version:", hex(version))
