from arduino_alvik import ArduinoAlvik
from time import sleep_ms


alvik = ArduinoAlvik()
alvik.begin()

while True:
    try:
        L, CL, C, CR, R = alvik.get_distance()
        T = alvik.get_distance_top()
        B = alvik.get_distance_bottom()
        print(f'T: {T} | B: {B} | L: {L} | CL: {CL} | C: {C} | CR: {CR} | R: {R}')
        sleep_ms(100)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        break
