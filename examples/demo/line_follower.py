from arduino_alvik import ArduinoAlvik
from time import sleep_ms


def calculate_center(left: int, center: int, right: int):
    centroid = 0
    sum_weight = left + center + right
    sum_values = left + 2 * center + 3 * right
    if sum_weight != 0:
        centroid = sum_values / sum_weight
        centroid = 2 - centroid
    return centroid


def run_line_follower(alvik):
    kp = 50.0
    line_sensors = alvik.get_line_sensors()
    print(f' {line_sensors}')

    error = calculate_center(*line_sensors)
    control = error * kp

    if control > 0.2:
        alvik.left_led.set_color(1, 0, 0)
        alvik.right_led.set_color(0, 0, 0)
    elif control < -0.2:
        alvik.left_led.set_color(1, 0, 0)
        alvik.right_led.set_color(0, 0, 0)
    else:
        alvik.left_led.set_color(0, 1, 0)
        alvik.right_led.set_color(0, 1, 0)

    alvik.set_wheels_speed(30 - control, 30 + control)
    sleep_ms(100)


if __name__ == "__main__":

    alvik = ArduinoAlvik()
    alvik.begin()

    alvik.left_led.set_color(0, 0, 1)
    alvik.right_led.set_color(0, 0, 1)

    while alvik.get_touch_ok():
        sleep_ms(50)

    while not alvik.get_touch_ok():
        sleep_ms(50)

    while True:
        try:
            while not alvik.get_touch_cancel():
                run_line_follower(alvik)

            while not alvik.get_touch_ok():
                alvik.left_led.set_color(0, 0, 1)
                alvik.right_led.set_color(0, 0, 1)
                alvik.brake()
                sleep_ms(100)

        except KeyboardInterrupt as e:
            print('over')
            alvik.stop()
            break
