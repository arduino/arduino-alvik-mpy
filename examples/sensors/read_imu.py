from arduino_alvik import ArduinoAlvik
from time import sleep_ms


alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        ax, ay, az = alvik.get_accelerations()
        gx, gy, gz = alvik.get_gyros()
        print(f'ax: {ax}, ay: {ay}, az: {az}, gx: {gx}, gy: {gy}, gz: {gz}')
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        break
