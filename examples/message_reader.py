from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
if alvik.begin() < 0:
    sys.exit()

speed = 0

while True:
    try:
        print(f'VER: {alvik.version}')
        print(f'LSP: {alvik.left_wheel.get_speed()}')
        print(f'RSP: {alvik.left_wheel.get_speed()}')
        print(f'TOUCH: {alvik.touch_bits}')
        print(f'RGB: {alvik.red} {alvik.green} {alvik.blue}')
        print(f'LINE: {alvik.left_line} {alvik.center_line} {alvik.right_line}')

        alvik.set_wheels_speed(speed, speed)
        speed = (speed + 1) % 60
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()

