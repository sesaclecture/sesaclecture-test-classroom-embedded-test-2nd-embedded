import RPi.GPIO as GPIO
from mfrc522 import SimpleMFRC522

reader = SimpleMFRC522()
try:
    print("태그를 대면 'KCCI'를 기록합니다...")
    reader.write("KCCI")
    print("기록 완료! 이제 다시 태그를 읽어봅니다...")
    uid, text = reader.read()
    print(f"UID={uid}  DATA='{text.strip()}'")
finally:
    GPIO.cleanup()
