from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:

        alvik.move(100.0)
        print("on target after move")

        alvik.move(50.0)
        print("on target after move")

        alvik.rotate(90.0)
        print("on target after rotation")

        alvik.rotate(-45.00)
        print("on target after rotation")

        x, y, theta = alvik.get_pose()
        print(f'Current pose is x={x}, y={y} ,theta={theta}')

        alvik.reset_pose(0, 0, 0)

        x, y, theta = alvik.get_pose()
        print(f'Updated pose is x={x}, y={y} ,theta={theta}')
        sleep_ms(500)

        print("___________NON-BLOCKING__________________")

        alvik.move(50.0, blocking=False)
        while not alvik.is_target_reached():
            print(f"Not yet on target received:{alvik.last_ack}")
        print("on target after move")

        alvik.rotate(45.0, blocking=False)
        while not alvik.is_target_reached():
            print(f"Not yet on target received:{alvik.last_ack}")
        print("on target after rotation")

        alvik.move(100.0, blocking=False)
        while not alvik.is_target_reached():
            print(f"Not yet on target received:{alvik.last_ack}")
        print("on target after move")

        alvik.rotate(-90.00, blocking=False)
        while not alvik.is_target_reached():
            print(f"Not yet on target received:{alvik.last_ack}")
        print("on target after rotation")

        x, y, theta = alvik.get_pose()
        print(f'Current pose is x={x}, y={y} ,theta={theta}')

        alvik.reset_pose(0, 0, 0)

        x, y, theta = alvik.get_pose()
        print(f'Updated pose is x={x}, y={y} ,theta={theta}')
        sleep_ms(500)

        alvik.stop()
        sys.exit()

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
