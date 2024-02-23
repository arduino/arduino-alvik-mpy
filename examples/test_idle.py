from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

speed = 0

while True:
    try:

        if alvik.is_on():
            print(f'VER: {alvik.version}')
            print(f'LSP: {alvik.left_wheel.get_speed()}')
            alvik.set_wheels_speed(speed, speed)
            speed = (speed + 1) % 30
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()

