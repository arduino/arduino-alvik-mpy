from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

speed = 0

while True:
    try:
        print(f'FW VER: {alvik.get_fw_version()}')
        print(f'LSP: {alvik.left_wheel.get_speed()}')
        print(f'RSP: {alvik.right_wheel.get_speed()}')
        print(f'LPOS: {alvik.left_wheel.get_position()}')
        print(f'RPOS: {alvik.right_wheel.get_position()}')
        print(f'TOUCH (UP): {alvik.get_touch_up()}')
        print(f'RGB: {alvik.get_color()}')
        print(f'LINE: {alvik.get_line_sensors()}')
        print(f'SOC: {alvik.get_battery_charge()}%')

        alvik.set_wheels_speed(speed, speed)
        speed = (speed + 1) % 60
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()

