#!/usr/bin/env python3
from time import sleep

from gpiozero import LED

led_r = LED(17)
led_g = LED(27)
led_b = LED(22)

leds = [led_r, led_g, led_b]

while True:
    for led in leds:
        led.on()
        sleep(0.5)

    for led in leds:
        led.off()
        sleep(0.5)
