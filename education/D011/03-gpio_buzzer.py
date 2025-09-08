#!/usr/bin/env python3
from time import sleep

from gpiozero import Buzzer

buzzer = Buzzer(18)

while True:
    buzzer.on()
    sleep(1)
    buzzer.off()
    sleep(2)
