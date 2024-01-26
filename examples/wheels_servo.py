from arduino_alvik import ArduinoAlvik
from time import sleep
import sys

alvik = ArduinoAlvik()
alvik.begin()

alvik.left_wheel.reset()
alvik.right_wheel.reset()

while True:
    try:
        alvik.left_wheel.set_position(30)
        sleep(2)
        print(f'Left wheel degs: {alvik.left_wheel.get_position()}')
        print(f'Right wheel degs: {alvik.right_wheel.get_position()}')

        alvik.right_wheel.set_position(10)
        sleep(2)
        print(f'Left wheel degs: {alvik.left_wheel.get_position()}')
        print(f'Right wheel degs: {alvik.right_wheel.get_position()}')

        alvik.left_wheel.set_position(180)
        sleep(2)
        print(f'Left wheel degs: {alvik.left_wheel.get_position()}')
        print(f'Right wheel degs: {alvik.right_wheel.get_position()}')

        alvik.right_wheel.set_position(270)
        sleep(2)
        print(f'Left wheel degs: {alvik.left_wheel.get_position()}')
        print(f'Right wheel degs: {alvik.right_wheel.get_position()}')

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
