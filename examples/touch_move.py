from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

alvik.left_led.set_color(1, 0, 0)
alvik.right_led.set_color(1, 0, 0)

movements = []


def blink():
    alvik.left_led.set_color(1, 0, 1)
    alvik.right_led.set_color(1, 0, 1)
    sleep_ms(200)
    alvik.left_led.set_color(1, 0, 0)
    alvik.right_led.set_color(1, 0, 0)


def add_movement():
    global movements

    if alvik.get_touch_up():
        movements.append('forward')
        blink()
        while alvik.get_touch_up():
            sleep_ms(100)
    if alvik.get_touch_down():
        movements.append('backward')
        blink()
        while alvik.get_touch_down():
            sleep_ms(100)
    if alvik.get_touch_left():
        movements.append('left')
        blink()
        while alvik.get_touch_left():
            sleep_ms(100)
    if alvik.get_touch_right():
        movements.append('right')
        blink()
        while alvik.get_touch_right():
            sleep_ms(100)
    if alvik.get_touch_cancel():
        movements = []
        for i in range(0, 3):
            val = i % 2
            alvik.left_led.set_color(val, 0, 0)
            alvik.right_led.set_color(val, 0, 0)
            sleep_ms(200)
        while alvik.get_touch_cancel():
            sleep_ms(100)


def run_movement(movement):
    if movement == 'forward':
        alvik.move(10, blocking=False)
    if movement == 'backward':
        alvik.move(-10, blocking=False)
    if movement == 'left':
        alvik.rotate(90, blocking=False)
    if movement == 'right':
        alvik.rotate(-90, blocking=False)
    while not alvik.get_touch_cancel() and not alvik.is_target_reached():
        alvik.left_led.set_color(1, 0, 0)
        alvik.right_led.set_color(1, 0, 0)
        sleep_ms(100)
        alvik.left_led.set_color(0, 0, 0)
        alvik.right_led.set_color(0, 0, 0)
        sleep_ms(100)

while alvik.get_touch_ok():
    sleep_ms(50)

while not (alvik.get_touch_ok() and len(movements) != 0):
    add_movement()
    sleep_ms(50)

try:
    while True:
        alvik.left_led.set_color(0, 0, 0)
        alvik.right_led.set_color(0, 0, 0)
        for move in movements:
            run_movement(move)
            if alvik.get_touch_cancel():
                break

        movements = []

        while not (alvik.get_touch_ok() and len(movements) != 0):
            alvik.left_led.set_color(1, 0, 0)
            alvik.right_led.set_color(1, 0, 0)
            alvik.brake()
            add_movement()
            sleep_ms(100)
except KeyboardInterrupt as e:
    print('over')
    alvik.stop()
    sys.exit()


