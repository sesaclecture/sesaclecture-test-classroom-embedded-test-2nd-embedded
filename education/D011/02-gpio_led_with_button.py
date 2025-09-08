#!/usr/bin/env python3
from signal import pause

from gpiozero import LED, Button

LED_PIN = 17
BTN_PIN = 27

led = LED(LED_PIN)
button = Button(BTN_PIN, bounce_time=0.05)

def toggle_led():
    led.toggle()

button.when_pressed = toggle_led

print("Press the button to toggle the LED.")
pause()
