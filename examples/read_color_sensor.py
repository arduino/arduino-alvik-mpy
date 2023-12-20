from arduino_robot import ArduinoRobot
from time import sleep_ms
import sys

robot = ArduinoRobot()

robot.run()
sleep_ms(1000)
robot.reset_hw()
speed = 0

while True:
    try:
        r, g, b = robot.get_color()
        print(f'RED: {r}, Green: {g}, Blue: {b}')
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        break
sys.exit()
