#!/usr/bin/env python3
from gpiozero import Button, Buzzer

buzzer = Buzzer(18)
button = Button(23)

while True:
    if button.is_pressed:
        buzzer.on()
    else:
        buzzer.off()
