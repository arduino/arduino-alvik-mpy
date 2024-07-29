from arduino_alvik import ArduinoAlvik
from time import sleep
import sys


def toggle_value():
    """
    This function yields a generator object that toggles values between 0 and 1.
    :return:
    """
    value = 0
    while True:
        yield value % 2
        value += 1


def toggle_left_led(custom_text: str, val) -> None:
    """
    This function toggles the lef led in the red channel. It also writes some custom text.
    :param custom_text: your custom text
    :param val: a toggle signal generator
    :return:
    """
    alvik.left_led.set_color(next(val), 0, 0)
    print(f"RED BLINKS! {custom_text}")


alvik = ArduinoAlvik()
alvik.timer('one_shot', 10000, toggle_left_led, ("10 seconds have passed... I won't do this again", toggle_value(), ))

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
