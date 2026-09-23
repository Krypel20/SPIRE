import RPi.GPIO as GPIO
import time

# GPIO w numeracji BCM
LED_GREEN_1 = 5    # pin fizyczny 29
LED_GREEN_2 = 6    # pin fizyczny 31
LED_GREEN_3 = 16   # pin fizyczny 36

GPIO.setmode(GPIO.BCM)

GPIO.setup(LED_GREEN_1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED_GREEN_2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(LED_GREEN_3, GPIO.OUT, initial=GPIO.LOW)


def blink(pin, times=5, interval=0.2):
    for _ in range(times):
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(interval)
        GPIO.output(pin, GPIO.LOW)
        time.sleep(interval)


try:
    # GPIO16 - światło stałe
    GPIO.output(LED_GREEN_3, GPIO.HIGH)

    # Miganie dwóch pozostałych
    while True:
        GPIO.output(LED_GREEN_1, GPIO.HIGH)
        GPIO.output(LED_GREEN_2, GPIO.LOW)
        time.sleep(0.3)

        GPIO.output(LED_GREEN_1, GPIO.LOW)
        GPIO.output(LED_GREEN_2, GPIO.HIGH)
        time.sleep(0.3)

except KeyboardInterrupt:
    pass

finally:
    GPIO.output(LED_GREEN_1, GPIO.LOW)
    GPIO.output(LED_GREEN_2, GPIO.LOW)
    GPIO.output(LED_GREEN_3, GPIO.LOW)
    GPIO.cleanup()