#!/usr/bin/env python3
from time import sleep

from gpiozero import LED, Button

LED_PINS = [17, 27, 22]
BTN_PINS = [5, 6, 13]

leds = [LED(p) for p in LED_PINS]
buttons = [Button(p, bounce_time=0.05) for p in BTN_PINS]

states = [False, False, False]

print("Press buttons to toggle corresponding LEDs (polling mode).")

while True:
    for i, btn in enumerate(buttons):
        if btn.is_pressed:
            states[i] = not states[i]
            if states[i]:
                leds[i].on()
            else:
                leds[i].off()

            while btn.is_pressed:
                sleep(0.01)
    sleep(0.01)
