#!/usr/bin/env python3
# RC522 + Raspberry Pi (SPI)
# - 태그가 감지되면 UID와 기본 블록(문자) 출력
# - Ctrl+C 로 종료

import time

import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522


def main():
    reader = SimpleMFRC522()  # CE0, RST=GPIO25 기본 사용
    print("RFID 리더 준비 완료. 카드를 리더에 갖다 대세요. (Ctrl+C 종료)")

    try:
        while True:
            # read() 는 태그가 올 때까지 블로킹 → 태그되면 (uid, text) 반환
            uid, text = reader.read()
            # uid: int, text: 기본 블록에 쓰인 문자열(없으면 빈 문자열)
            print(f"[TAG] UID={uid}  DATA='{text.strip()}'")
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
        print("종료")

if __name__ == "__main__":
    main()
