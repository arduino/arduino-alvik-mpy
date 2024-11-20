from time import sleep_ms

from arduino import start
from arduino_alvik import ArduinoAlvik


alvik = ArduinoAlvik()


def setup():
    alvik.begin()


def loop():
    print('blinking LEDs')
    alvik.left_led.set_color(0, 0, 1)
    alvik.right_led.set_color(0, 0, 1)
    sleep_ms(500)
    alvik.left_led.set_color(1, 0, 0)
    alvik.right_led.set_color(1, 0, 0)
    sleep_ms(500)


start(setup, loop)
