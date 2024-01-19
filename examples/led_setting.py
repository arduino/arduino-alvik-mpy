from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()

alvik.run()
sleep_ms(100)
alvik.reset_hw()

while True:
    try:
        #alvik._set_leds(0xff)
        #sleep_ms(1000)
        #alvik._set_leds(0x00)
        #sleep_ms(1000)
        alvik.set_builtin_led(1)
        sleep_ms(1000)
        alvik.set_illuminator(1)
        sleep_ms(1000)
        alvik.set_builtin_led(0)
        sleep_ms(1000)
        alvik.set_illuminator(0)
        sleep_ms(1000)
        alvik.set_left_led_color(0,0,1)
        sleep_ms(1000)
        alvik.set_right_led_color(0,0,1)
        sleep_ms(1000)
        alvik.set_left_led_color(0,1,0)
        sleep_ms(1000)
        alvik.set_right_led_color(0,1,0)
        sleep_ms(1000)
        alvik.set_left_led_color(1,0,0)
        sleep_ms(1000)
        alvik.set_right_led_color(1,0,0)
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        sys.exit()
