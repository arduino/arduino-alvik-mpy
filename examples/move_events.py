from arduino_alvik import ArduinoAlvik
from time import sleep
import sys

value = 0


def toggle_left_led(custom_text: str = '') -> None:
    global value
    value = (value + 1) % 2
    alvik.left_led.set_color(value, 0, 0)
    print(f"RED BLINKS! {custom_text}")


def simple_print(custom_text: str = '') -> None:
    print(custom_text)

alvik = ArduinoAlvik()
alvik.on_shake(toggle_left_led, ("ALVIK WAS SHAKEN... YOU MAKE ME SHIVER :)", ))
alvik.on_x_tilt(simple_print, ("TILTED ON X",))
alvik.on_nx_tilt(simple_print, ("TILTED ON -X",))
alvik.on_y_tilt(simple_print, ("TILTED ON Y",))
alvik.on_ny_tilt(simple_print, ("TILTED ON -Y",))
alvik.on_z_tilt(simple_print, ("TILTED ON Z",))
alvik.on_nz_tilt(simple_print, ("TILTED ON -Z",))

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
