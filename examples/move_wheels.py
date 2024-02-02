from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        alvik.left_wheel.set_speed(10)
        sleep_ms(1000)
        print(f'LSP: {alvik.left_wheel.get_speed()}')
        print(f'RSP: {alvik.right_wheel.get_speed()}')

        alvik.right_wheel.set_speed(10)
        sleep_ms(1000)

        print(f'LSP: {alvik.left_wheel.get_speed()}')
        print(f'RSP: {alvik.right_wheel.get_speed()}')

        alvik.left_wheel.set_speed(20)
        sleep_ms(1000)

        print(f'LSP: {alvik.left_wheel.get_speed()}')
        print(f'RSP: {alvik.right_wheel.get_speed()}')

        alvik.right_wheel.set_speed(20)
        sleep_ms(1000)

        print(f'LSP: {alvik.left_wheel.get_speed()}')
        print(f'RSP: {alvik.right_wheel.get_speed()}')

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
