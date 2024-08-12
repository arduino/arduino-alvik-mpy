from arduino_alvik import ArduinoAlvik
from time import sleep, sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

alvik.left_wheel.reset()
alvik.right_wheel.reset()

while True:
    try:

        alvik.set_wheels_position(45, 45)
        sleep(2)
        print(f'Left wheel degs: {alvik.left_wheel.get_position()}')
        print(f'Right wheel degs: {alvik.right_wheel.get_position()}')

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

        print("___________NON-BLOCKING__________________")

        alvik.set_wheels_position(90, 90, blocking=False)
        while not alvik.is_target_reached():
            alvik.left_led.set_color(1, 0, 0)
            sleep_ms(500)
            alvik.left_led.set_color(0, 0, 0)
            sleep_ms(500)
        print(f'Wheels position reached: R{alvik.right_wheel.get_position()} L:{alvik.left_wheel.get_position()}')

        alvik.left_wheel.set_position(180)
        while not alvik.left_wheel.is_target_reached():
            alvik.left_led.set_color(1, 0, 0)
            sleep_ms(500)
            alvik.left_led.set_color(0, 0, 0)
            sleep_ms(500)
        print(f'Left wheel position reached: {alvik.left_wheel.get_position()}')

        alvik.right_wheel.set_position(180)
        while not alvik.right_wheel.is_target_reached():
            alvik.right_led.set_color(1, 0, 0)
            sleep_ms(500)
            alvik.right_led.set_color(0, 0, 0)
            sleep_ms(500)
        print(f'Left wheel position reached: {alvik.right_wheel.get_position()}')

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
