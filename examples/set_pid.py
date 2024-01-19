from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

robot = ArduinoAlvik()

robot.run()
sleep_ms(1000)
robot.reset_hw()
speed = 0

while True:
    try:
        robot.set_pid('L', 10.0, 1.3, 4.2)
        sleep_ms(100)
        robot.set_pid('R', 4.0, 13, 1.9)
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        break
sys.exit()
