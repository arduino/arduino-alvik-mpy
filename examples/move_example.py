from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        alvik.set_wheels_speed(10, 10)
        sleep_ms(1000)

        alvik.set_wheels_speed(30, 60)
        sleep_ms(1000)

        alvik.set_wheels_speed(60, 30)
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
