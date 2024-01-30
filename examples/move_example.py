from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        alvik.move(100.0)
        while (ack := alvik.get_ack()) != ord('M'):
            print(f'moving... not on target yet ack={ack}')
            sleep_ms(200)
        print("on target after move")

        alvik.rotate(90.0)
        while (ack := alvik.get_ack()) != ord('R'):
            print(f'rotating... not on target yet ack={ack}')
            sleep_ms(200)
        print("on target after rotation")

    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()
