from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        alvik.set_speeds(10, 10)
        sleep_ms(1000)

        alvik.set_speeds(30, 60)
        sleep_ms(1000)

        alvik.set_speeds(60, 30)
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        alvik.set_speeds(0, 0)
        sys.exit()
