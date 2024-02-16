from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        r, g, b = alvik.get_color()
        h, s, v = alvik.get_color('hsv')
        print(f'RED: {r}, Green: {g}, Blue: {b}, HUE: {h}, SAT: {s}, VAL: {v}')
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
