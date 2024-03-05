from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

menu_status = 0


def update_led_status(val):
    if val == 0:
        alvik.left_led.set_color(0, 0, 1)
        alvik.right_led.set_color(0, 0, 1)
    elif val == 1:
        alvik.left_led.set_color(0, 1, 0)
        alvik.right_led.set_color(0, 1, 0)
    elif val == -1:
        alvik.left_led.set_color(1, 0, 0)
        alvik.right_led.set_color(1, 0, 0)


while True:

    update_led_status(menu_status)

    try:

        if alvik.get_touch_ok():
            if menu_status == 0:
                import line_follower
            elif menu_status == 1:
                import hand_follower
            elif menu_status == -1:
                import touch_move

        if alvik.get_touch_up() and menu_status < 1:
            menu_status += 1
            update_led_status(menu_status)
            while alvik.get_touch_up():
                sleep_ms(100)
        if alvik.get_touch_down() and menu_status > -1:
            menu_status -= 1
            update_led_status(menu_status)
            while alvik.get_touch_down():
                sleep_ms(100)

        sleep_ms(100)

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
