from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

reference = 10.0

while True:
    try:
        L, CL, C, CR, R = alvik.get_distance()
        print(f'C: {C}')
        error = C - reference
        alvik.set_wheels_speed(error*10, error*10)
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()