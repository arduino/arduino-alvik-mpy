from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        alvik.set_behaviour(behaviour=1)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
