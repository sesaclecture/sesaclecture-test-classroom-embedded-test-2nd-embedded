#!/usr/bin/env python3
# 0.42" SSD1306 (72x40) + Button
# - short press: 다음 메뉴
# - long press(>= HOLD_SEC): 현재 메뉴 가격 표시 (누르는 동안만), release 시 메뉴명 복귀

from time import monotonic, sleep, strftime

from gpiozero import Button
from luma.core.interface.serial import i2c
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

I2C_ADDR = 0x3C          # i2cdetect -y 1 결과에 맞게 0x3C/0x3D
WIDTH, HEIGHT = 72, 40   # 0.42" 보드 해상도
ROTATE = 0               # 뒤집혔으면 2

BTN_PIN = 27             # 버튼 GPIO (BCM)
DEBOUNCE = 0.05          # 50ms 디바운스
HOLD_SEC = 1.0           # 1초 이상이면 '길게 누름'

# 메뉴 샘플
MENU = [
    {"name": "Americano", "price": 2000},
    {"name": "Latte",     "price": 3000},
    {"name": "Mocha",     "price": 3500},
    {"name": "Tea",       "price": 1500},
]

# ------- OLED 도우미 -------
def center_text(draw, text, y, font):
    w, h = draw.textsize(text, font=font)
    x = max(0, (WIDTH - w) // 2)
    draw.text((x, y), text, font=font, fill=255)

def show_menu_name(dev, font, idx):
    img = Image.new("1", (WIDTH, HEIGHT))
    d = ImageDraw.Draw(img)
    center_text(d, "MENU", 2, font)
    center_text(d, MENU[idx]["name"][:12], 20, font)  # 너무 길면 잘림
    dev.display(img)

def show_price(dev, font, idx):
    img = Image.new("1", (WIDTH, HEIGHT))
    d = ImageDraw.Draw(img)
    center_text(d, MENU[idx]["name"][:12], 2, font)
    center_text(d, f"{MENU[idx]['price']} won", 20, font)
    dev.display(img)

def main():
    # OLED
    serial = i2c(port=1, address=I2C_ADDR)
    dev = ssd1306(serial, width=WIDTH, height=HEIGHT, rotate=ROTATE)
    font = ImageFont.load_default()

    # 버튼
    btn = Button(BTN_PIN, bounce_time=DEBOUNCE)
    press_t = {"t": None}      # 누른 시각 저장
    state = {"idx": 0, "showing_price": False}

    # 초기 화면
    show_menu_name(dev, font, state["idx"])

    def on_pressed():
        press_t["t"] = monotonic()
        # 길게 누를 것 같으면 가격으로 전환을 미리 예약하지 않고,
        # when_held 대신 release 시점에서 길이로 판정해 깔끔히 처리

    def on_released():
        if press_t["t"] is None:
            return
        dt = monotonic() - press_t["t"]
        press_t["t"] = None

        if dt >= HOLD_SEC:
            # 길게 눌렀던 경우: 가격을 보여준 상태였다면 이름으로 복귀
            # (누르는 동안 가격, 떼면 메뉴명 복귀)
            state["showing_price"] = False
            show_menu_name(dev, font, state["idx"])
        else:
            # 짧게 눌렀던 경우: 다음 메뉴로 이동
            state["idx"] = (state["idx"] + 1) % len(MENU)
            show_menu_name(dev, font, state["idx"])

    # 길게 누르는 동안 가격을 ‘실시간’으로 보여주기 위한 폴링 루프
    # (when_held 대신: 디바운스·타이밍 엣지케이스 줄이고 직관 유지)
    btn.when_pressed = on_pressed
    btn.when_released = on_released

    try:
        while True:
            if press_t["t"] is not None and not state["showing_price"]:
                # 누른 지 HOLD_SEC 지났으면 가격 표시 시작
                if (monotonic() - press_t["t"]) >= HOLD_SEC:
                    state["showing_price"] = True
                    show_price(dev, font, state["idx"])
            sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        dev.clear()

if __name__ == "__main__":
    main()
