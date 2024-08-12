from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

reference = 10.0

alvik.left_led.set_color(0, 1, 0)
alvik.right_led.set_color(0, 1, 0)

while alvik.get_touch_ok():
    sleep_ms(50)

while not alvik.get_touch_ok():
    sleep_ms(50)

try:
    while True:
        while not alvik.get_touch_cancel():
            alvik.left_led.set_color(0, 0, 0)
            alvik.right_led.set_color(0, 0, 0)
            L, CL, C, CR, R = alvik.get_distance()
            print(f'C: {C}')
            error = C - reference
            alvik.set_wheels_speed(error*10, error*10)
            sleep_ms(100)

        while not alvik.get_touch_ok():
            alvik.left_led.set_color(0, 1, 0)
            alvik.right_led.set_color(0, 1, 0)
            alvik.brake()
            sleep_ms(100)
except KeyboardInterrupt as e:
    print('over')
    alvik.stop()
    sys.exit()
