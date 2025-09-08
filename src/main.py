#!/usr/bin/env python3
# pylint: disable=import-error

"""
GPIO 과제
- gpiozero 라이브러리 사용

UART 과제
- pyserial 라이브러리 사용
- UART 통신: 115200 bps, 8N1, 플로우 제어 없음
- TXD3/RXD3을 활용
"""

import sys
import time

from gpiozero import LED, Button
from serial import Serial


def blink_led() -> None:
    """
    [문제 1] 18번 핀에 연결된 LED를 1초 간격으로 ON/OFF를 10번 반복
    - gpiozero.LED 사용
    - 종료시 LED는 OFF 상태
    """
    led = LED(18)
    for _ in range(10):
        led.on()
        time.sleep(1)
        led.off()
        time.sleep(1)


def check_to_input_button() -> None:
    """
    [문제 2] 18번 핀 버튼 입력을 받아서
      - 눌렸을 때: 'pressed'
      - 뗐을 때:   'released'
    를 출력한다.
    - polling 방식으로 구현할 것.
    - 버튼 입력을 10번 받았으면 종료.
    """
    btn = Button(18, pull_up=True)

    cnt = 0
    prev = btn.is_pressed
    while cnt < 10:
        cur = btn.is_pressed
        if cur != prev:
            if cur:
                print("pressed", file=sys.stdout)
                cnt += 1
            else:
                print("released", file=sys.stdout)
            prev = cur


def blink_led_through_button() -> None:
    """
    [문제 3]
    - 12번: LED 출력
    - 13번: Button 입력
    - 버튼이 눌려 있는 동안에만 LED가 0.5초 간격으로 깜빡인다.
    - 버튼이 10번 눌려졌으면 종료.
    - 종료시 LED는 OFF 상태
    """
    led = LED(12)
    btn = Button(13, pull_up=True)

    cnt = 0
    while cnt < 10:
        if btn.is_pressed:
            cnt += 1
            led.on()
            time.sleep(0.5)
            led.off()
            time.sleep(0.5)
        else:
            led.off()
            time.sleep(0.005)


def transmit_msg() -> None:
    """
    [문제 1] UART3로 "Hello World! {i}" 문자열을 1초마다 전송
    - 총 10번 전송 후 종료
    - 개행을 붙여 전송 (수신/테스트 편의)
    """
    ser = Serial("/dev/ttyAMA3", baudrate=115200, timeout=0.1)
    try:
        for i in range(10):
            msg = f"Hello World! {i}\n".encode()
            ser.write(msg)
            time.sleep(1)
    finally:
        ser.close()


def receive_msg() -> None:
    """
    [문제 2] UART3에서 줄 단위로 읽어 화면에 출력.
    - 'exit' (대소문자 무시) 라인을 수신하면 함수 종료
    """
    ser = Serial("/dev/ttyAMA3", baudrate=115200, timeout=0.1)
    try:
        buf = bytearray()
        while True:
            ch = ser.read(1)
            if not ch:
                continue
            if ch == b"\n":
                line = buf.decode(errors="ignore").rstrip("\r")
                print(line, file=sys.stdout)
                if line.strip().lower() == "exit":
                    break
                buf.clear()
            else:
                buf.extend(ch)
    finally:
        ser.close()


if __name__ == "__main__":
    blink_led()
    check_to_input_button()
    blink_led_through_button()

    transmit_msg()
    receive_msg()
