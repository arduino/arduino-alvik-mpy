from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        alvik.set_builtin_led(1)
        sleep_ms(1000)
        alvik.set_illuminator(1)
        sleep_ms(1000)
        alvik.set_builtin_led(0)
        sleep_ms(1000)
        alvik.set_illuminator(0)
        sleep_ms(1000)
        alvik.left_led.set_color(0, 0, 1)
        sleep_ms(1000)
        alvik.left_led.set_color(0, 1, 0)
        sleep_ms(1000)
        alvik.left_led.set_color(1, 0, 0)
        sleep_ms(1000)
        alvik.left_led.set_color(1, 1, 1)
        sleep_ms(1000)
        alvik.right_led.set_color(0, 0, 1)
        sleep_ms(1000)
        alvik.right_led.set_color(0, 1, 0)
        sleep_ms(1000)
        alvik.right_led.set_color(1, 0, 0)
        sleep_ms(1000)
        alvik.right_led.set_color(1, 1, 1)
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
