from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:

        # -- LINEAR MOVEMENTS --

        print("Move fw 0.05 m")
        alvik.move(0.05, unit='m')
        sleep_ms(2000)

        print("Move fw 10 cm")
        alvik.move(5, unit='cm')
        sleep_ms(2000)

        print("Move bw 100 mm")
        alvik.move(-100, unit='mm')
        sleep_ms(2000)

        print("Move fw 1 inch")
        alvik.move(1, unit='in')
        sleep_ms(2000)

        # -- WHEEL ROTATIONS --
        alvik.right_wheel.reset()
        sleep_ms(2000)
        curr_pos = alvik.right_wheel.get_position()
        print(f'R wheel pos: {curr_pos}')
        sleep_ms(2000)

        print("Rotate right wheel 25% fw")
        alvik.right_wheel.set_position(25, unit='%')
        sleep_ms(2000)
        curr_pos = alvik.right_wheel.get_position()
        print(f'R wheel pos: {curr_pos}')

        print("Rotate right wheel 90 deg bw")
        alvik.right_wheel.set_position(-90, unit='deg')
        sleep_ms(2000)
        curr_pos = alvik.right_wheel.get_position()
        print(f'R wheel pos: {curr_pos}')

        print("Rotate right wheel pi rad fw")
        alvik.right_wheel.set_position(3.14, unit='rad')
        sleep_ms(2000)
        curr_pos = alvik.right_wheel.get_position()
        print(f'R wheel pos: {curr_pos}')

        print("Rotate right wheel a quarter revolution bw")
        alvik.right_wheel.set_position(-0.25, unit='rev')
        sleep_ms(2000)
        curr_pos = alvik.right_wheel.get_position()
        print(f'R wheel pos: {curr_pos}')

        # -- WHEELS SPEED --
        print("Set speed 12 rpm (1 rev in 5 sec)")
        alvik.set_wheels_speed(12, 12, 'rpm')
        sleep_ms(1000)
        print(f"Current speed is {alvik.get_wheels_speed()} rpm")

        print("Set speed -pi rad/s (1 back rev in 2 sec)")
        alvik.set_wheels_speed(-3.1415, -3.1415, 'rad/s')
        sleep_ms(1000)
        print(f"Current speed is {alvik.get_wheels_speed()} rpm")

        print("Set speed 180 deg/s (1 back rev in 2 sec)")
        alvik.set_wheels_speed(180, 180, 'deg/s')
        sleep_ms(1000)
        print(f"Current speed is {alvik.get_wheels_speed()} rpm")


        # -- DRIVE --
        print("Driving at 10 mm/s (expecting approx 5.6 rpm)")
        alvik.drive(10, 0, linear_unit='mm/s')
        sleep_ms(2000)
        print(f"Current speed is {alvik.get_wheels_speed()} rpm")

        print("Driving at 2 cm/s (expecting approx 11.2 rpm)")
        alvik.drive(2, 0, linear_unit='cm/s')
        sleep_ms(2000)
        print(f"Current speed is {alvik.get_wheels_speed()} rpm")

        print("Driving at 1 in/s (expecting approx 14 rpm)")
        alvik.drive(1, 0, linear_unit='in/s')
        sleep_ms(2000)
        print(f"Current speed is {alvik.get_wheels_speed()} rpm")

        print("Driving at 5 mm/s (expecting approx 5.6 rpm) pi/8 rad/s (22.5 deg/s)")
        alvik.drive(5, 3.1415/8, linear_unit='mm/s', angular_unit='rad/s')
        sleep_ms(2000)
        print(f"Current speed is {alvik.get_drive_speed()} rpm")

        alvik.stop()
        sys.exit()

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
