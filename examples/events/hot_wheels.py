from arduino_alvik import ArduinoAlvik
from time import sleep_ms


def stop_when_up(alvik):
    print("lift")
    alvik.set_wheels_speed(0, 0)


def run_when_down(alvik):
    print("drop")
    alvik.set_wheels_speed(20, 20)


alvik = ArduinoAlvik()
alvik.on_lift(stop_when_up, (alvik,))
alvik.on_drop(run_when_down, (alvik,))
alvik.begin()
color_val = 0


def blinking_leds(val):
    alvik.left_led.set_color(val & 0x01, val & 0x02, val & 0x04)
    alvik.right_led.set_color(val & 0x02, val & 0x04, val & 0x01)


while not alvik.get_touch_ok():
    sleep_ms(100)

alvik.set_wheels_speed(20, 20)

while not alvik.get_touch_cancel():

    try:
        blinking_leds(color_val)
        color_val = (color_val + 1) % 7
        sleep_ms(500)

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        break

alvik.stop()
