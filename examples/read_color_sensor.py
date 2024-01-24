from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()
speed = 0

while True:
    try:
        r, g, b = alvik.get_color()
        print(f'RED: {r}, Green: {g}, Blue: {b}')
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        break
sys.exit()
