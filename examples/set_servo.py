from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

i = 0

while True:
    try:

        alvik.servo_A.set_position(i)
        alvik.servo_B.set_position(i)

        i = (i + 1) % 180

        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
