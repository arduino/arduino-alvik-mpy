from arduino_alvik import ArduinoAlvik
from time import sleep_ms

from line_follower import run_line_follower
from touch_move import run_touch_move
from hand_follower import run_hand_follower


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
            while not alvik.get_touch_cancel():
                if menu_status == 0:
                    run_line_follower(alvik)
                elif menu_status == 1:
                    run_hand_follower(alvik)
                elif menu_status == -1:
                    if run_touch_move(alvik) < 0:
                        break
            alvik.brake()

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
        break
