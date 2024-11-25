from arduino_alvik import ArduinoAlvik
from time import sleep_ms


alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        if alvik.get_touch_any():
            if alvik.get_touch_up():
                print("UP")
            if alvik.get_touch_down():
                print("DOWN")
            if alvik.get_touch_left():
                print("LEFT")
            if alvik.get_touch_right():
                print("RIGHT")
            if alvik.get_touch_ok():
                print("OK")
            if alvik.get_touch_cancel():
                print("CANCEL")
            if alvik.get_touch_center():
                print("CENTER")

        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        break
