from arduino_alvik import ArduinoAlvik
from time import sleep_ms


alvik = ArduinoAlvik()
alvik.begin()

try:

    alvik.move(100.0, 'mm')
    print("on target after move")

    alvik.move(50.0, 'mm')
    print("on target after move")

    alvik.rotate(90.0, 'deg')
    print("on target after rotation")

    alvik.rotate(-45.00, 'deg')
    print("on target after rotation")

    x, y, theta = alvik.get_pose()
    print(f'Current pose is x(cm)={x}, y(cm)={y}, theta(deg)={theta}')

    alvik.reset_pose(0, 0, 0)

    x, y, theta = alvik.get_pose()
    print(f'Updated pose is x(cm)={x}, y(cm)={y}, theta(deg)={theta}')
    sleep_ms(500)

    print("___________NON-BLOCKING__________________")

    alvik.move(50.0, 'mm', blocking=False)

    while not alvik.is_target_reached():
        alvik.left_led.set_color(1, 0, 0)
        sleep_ms(500)
        alvik.left_led.set_color(0, 0, 0)
        sleep_ms(500)
    print("on target after move")

    alvik.rotate(45.0, 'deg', blocking=False)
    while not alvik.is_target_reached():
        alvik.left_led.set_color(1, 0, 0)
        sleep_ms(500)
        alvik.left_led.set_color(0, 0, 0)
        sleep_ms(500)
    print("on target after rotation")

    alvik.move(100.0, 'mm', blocking=False)
    while not alvik.is_target_reached():
        alvik.left_led.set_color(1, 0, 0)
        sleep_ms(500)
        alvik.left_led.set_color(0, 0, 0)
        sleep_ms(500)
    print("on target after move")

    alvik.rotate(-90.00, 'deg', blocking=False)
    while not alvik.is_target_reached():
        alvik.left_led.set_color(1, 0, 0)
        sleep_ms(500)
        alvik.left_led.set_color(0, 0, 0)
        sleep_ms(500)
    print("on target after rotation")

    x, y, theta = alvik.get_pose()
    print(f'Current pose is x(cm)={x}, y(cm)={y}, theta(deg)={theta}')

    alvik.reset_pose(0, 0, 0)

    x, y, theta = alvik.get_pose()
    print(f'Updated pose is x={x}, y={y}, theta(deg)={theta}')
    sleep_ms(500)

except KeyboardInterrupt as e:
    print('Test interrupted')

finally:
    alvik.stop()
    print("END of pose example")