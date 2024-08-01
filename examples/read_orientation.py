from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        roll, pitch, yaw = alvik.get_orientation()
        print(f'ROLL: {roll}, PITCH: {pitch}, YAW: {yaw}')
        sleep_ms(50)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
