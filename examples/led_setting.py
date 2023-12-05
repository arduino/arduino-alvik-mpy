from arduino_robot import ArduinoRobot
from time import sleep_ms
import sys

robot = ArduinoRobot()


while True:
    try:
        robot.set_leds(0xf0)
        sleep_ms(1000)
        robot.set_leds(0x0f)
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        robot.set_speed(0, 0)
        sys.exit()
