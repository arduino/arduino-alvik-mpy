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
        if robot.get_touch_any():
            if robot.get_touch_up():
                print("UP")
            if robot.get_touch_down():
                print("DOWN")
            if robot.get_touch_left():
                print("LEFT")
            if robot.get_touch_right():
                print("RIGHT")
            if robot.get_touch_ok():
                print("OK")
            if robot.get_touch_cancel():
                print("CANCEL")
            if robot.get_touch_center():
                print("CENTER")

        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        break
sys.exit()
