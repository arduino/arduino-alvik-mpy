from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:

        alvik.move(100.0)
        while (ack := alvik.get_ack()) != ord('M'):
            sleep_ms(200)
        print("on target after move")

        alvik.rotate(90.0)
        while (ack := alvik.get_ack()) != ord('R'):
            sleep_ms(200)
        print("on target after rotation")

        alvik.move(50.0)
        while (ack := alvik.get_ack()) != ord('M'):
            sleep_ms(200)
        print("on target after move")

        alvik.rotate(-45.00)
        while (ack := alvik.get_ack()) != ord('R'):
            sleep_ms(200)
        print("on target after rotation")

        x, y, theta = alvik.get_pose()
        print(f'Current pose is x={x}, y={y} ,theta={theta}')

        alvik.reset_pose(0, 0, 0)

        x, y, theta = alvik.get_pose()
        print(f'Updated pose is x={x}, y={y} ,theta={theta}')
        sleep_ms(500)

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
