from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()
speed = 0

while True:
    try:
        alvik.left_wheel.set_pid_gains(10.0, 1.3, 4.2)
        sleep_ms(100)
        alvik.right_wheel.set_pid_gains(4.0, 13, 1.9)
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
