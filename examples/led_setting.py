from arduino_robot import ArduinoRobot
from time import sleep_ms
import sys

robot = ArduinoRobot()


while True:
    try:
        robot.set_leds(0xff)
        sleep_ms(1000)
        robot.set_leds(0x00)
        sleep_ms(1000)
        robot.set_builtin_led(1)
        sleep_ms(1000)
        robot.set_illuminator(1)
        sleep_ms(1000)
        robot.set_builtin_led(0)
        sleep_ms(1000)
        robot.set_illuminator(0)
        sleep_ms(1000)
        robot.set_left_led_color(0,0,1)
        sleep_ms(1000)
        robot.set_right_led_color(0,0,1)
        sleep_ms(1000)
        robot.set_left_led_color(0,1,0)
        sleep_ms(1000)
        robot.set_right_led_color(0,1,0)
        sleep_ms(1000)
        robot.set_left_led_color(1,0,0)
        sleep_ms(1000)
        robot.set_right_led_color(1,0,0)
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        robot.set_speed(0, 0)
        sys.exit()
