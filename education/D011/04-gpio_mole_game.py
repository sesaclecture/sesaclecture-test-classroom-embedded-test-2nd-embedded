#!/usr/bin/env python3
from random import uniform
from time import sleep

from gpiozero import LED, Button, Buzzer

LED_PIN    = 17
BUTTON_PIN = 27
BUZZ_PIN   = 18

led = LED(LED_PIN)
btn = Button(BUTTON_PIN, bounce_time=0.05)
buz = Buzzer(BUZZ_PIN)

score = 0
print("두더지 게임 시작! LED가 켜지면 0.5초 안에 버튼을 누르세요. (Ctrl+C 종료)")

try:
    while True:
        sleep(uniform(0.6, 1.6))
        led.on()

        pressed = btn.wait_for_press(timeout=0.5)
        led.off()

        if pressed:
            buz.on()
            sleep(0.2)
            buz.off()
            score += 1
            print(f"잡았다! 현재 점수: {score}")

            btn.wait_for_release(timeout=0.5)
        else:
            print("놓쳤다!")

        sleep(0.2)

except KeyboardInterrupt:
    pass
finally:
    led.off()
    buz.off()
    print("\n게임 종료!")
