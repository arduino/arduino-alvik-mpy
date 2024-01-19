from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

robot = ArduinoAlvik()

robot.run()
sleep_ms(100)
robot.reset_hw()
speed = 0

while True:
    try:
        print(f'VER: {robot.version}')
        print(f'LSP: {robot.l_speed}')
        print(f'RSP: {robot.r_speed}')
        print(f'TOUCH: {robot.touch_bits}')
        print(f'RGB: {robot.red} {robot.green} {robot.blue}')
        print(f'LINE: {robot.left_line} {robot.center_line} {robot.right_line}')

        robot.set_speeds(speed, speed)
        speed = (speed + 1) % 60
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        robot.set_speeds(0, 0)
        break
sys.exit()
