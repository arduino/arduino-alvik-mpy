from arduino_robot import ArduinoRobot
from time import sleep_ms
import sys

robot = ArduinoRobot()

robot.run()
sleep_ms(100)
robot.reset()
speed = 0

while True:
    try:
        print(f'VER: {robot.version}')
        print(f'LSP: {robot.l_speed}')
        print(f'RSP: {robot.r_speed}')
        print(f'TOUCH: {robot.touch_bits}')
        print(f'RGB: {robot.red} {robot.green} {robot.blue}')
        print(f'LINE: {robot.left_line} {robot.center_line} {robot.right_line}')

        robot.set_speed(speed, speed)
        speed = (speed + 1) % 60
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        robot.set_speed(0, 0)
        sys.exit()
