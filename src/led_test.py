import RPi.GPIO as GPIO
import time

# GPIO - numeracja BCM
RED = 17
GREEN = 27
BLUE = 22

GPIO.setmode(GPIO.BCM)

GPIO.setup(RED, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(GREEN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(BLUE, GPIO.OUT, initial=GPIO.LOW)


def set_rgb(r, g, b):
    GPIO.output(RED, r)
    GPIO.output(GREEN, g)
    GPIO.output(BLUE, b)


try:
    print("Test RGB HW-479")

    # 1. Zgaszona
    print("OFF")
    set_rgb(0, 0, 0)
    time.sleep(1)

    # 2. Czerwony
    print("RED - błąd krytyczny")
    set_rgb(1, 0, 0)
    time.sleep(2)

    # 3. Żółty / bursztynowy
    print("YELLOW - rozruch / oczekiwanie")
    set_rgb(1, 1, 0)
    time.sleep(2)

    # 4. Niebieski
    print("BLUE - stabilizacja aktywna")
    set_rgb(0, 0, 1)
    time.sleep(2)

    # 5. Zielony
    print("GREEN - stabilizacja w spoczynku")
    set_rgb(0, 1, 0)
    time.sleep(2)

    # 6. Biały
    print("WHITE - autotest")
    set_rgb(1, 1, 1)
    time.sleep(2)

    # Koniec testu
    print("OFF")
    set_rgb(0, 0, 0)

except KeyboardInterrupt:
    print("\nPrzerwano.")

finally:
    set_rgb(0, 0, 0)
    GPIO.cleanup()