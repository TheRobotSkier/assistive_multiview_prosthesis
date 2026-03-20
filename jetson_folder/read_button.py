import os
os.environ["JETSON_MODEL_NAME"] = "JETSON_ORIN_NANO"

import time
import Jetson.GPIO as GPIO

BUTTON_PIN = 18  # physical pin 18

GPIO.setmode(GPIO.BOARD)
GPIO.setup(BUTTON_PIN, GPIO.IN)

try:
    while True:
        value = GPIO.input(BUTTON_PIN)
        print(value)
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    GPIO.cleanup()