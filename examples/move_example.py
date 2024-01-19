from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

robot = ArduinoAlvik()

robot.run()
sleep_ms(100)
robot.reset_hw()


while True:
    try:
        robot.set_speed(10, 10)
        sleep_ms(1000)

        robot.set_speed(30, 60)
        sleep_ms(1000)

        robot.set_speed(60, 30)
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        robot.set_speed(0, 0)
        sys.exit()
