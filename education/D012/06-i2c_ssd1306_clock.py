#!/usr/bin/env python3
# 0.42" SSD1306 (72x40) clock for Raspberry Pi
# I2C addr 0x3C 기준, 기본 폰트로 2줄(시:분:초 / 월-일) 표시

from time import sleep, strftime

from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

I2C_ADDR = 0x3C          # 보드에 따라 0x3D일 수도 있음 (i2cdetect -y 1 로 확인)
WIDTH, HEIGHT = 72, 40   # 0.42" 모듈 해상도
ROTATE = 0               # 필요 시 0/2 등으로 조정

def draw_centered(draw, text, y, font):
    w, h = draw.textsize(text, font=font)
    x = max(0, (WIDTH - w) // 2)
    draw.text((x, y), text, font=font, fill=255)

def main():
    serial = i2c(port=1, address=I2C_ADDR)
    dev = ssd1306(serial, width=WIDTH, height=HEIGHT, rotate=ROTATE)

    font = ImageFont.load_default()  # 내장 기본 폰트(약 8x8 픽셀)

    try:
        while True:
            # 새 프레임 버퍼
            img = Image.new("1", (WIDTH, HEIGHT))
            d = ImageDraw.Draw(img)

            # 내용: 1줄 = HH:MM:SS (최대 8글자), 2줄 = MM/DD
            line1 = strftime("%H:%M:%S")
            line2 = strftime("%m/%d")

            # 대략 줄 간격 16px 권장 (가독성)
            draw_centered(d, line1, 4, font)      # 상단
            draw_centered(d, line2, 22, font)     # 하단

            dev.display(img)
            sleep(0.2)  # 5 Hz 업데이트
    except KeyboardInterrupt:
        pass
    finally:
        dev.clear()

if __name__ == "__main__":
    main()
