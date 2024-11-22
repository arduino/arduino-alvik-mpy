from time import sleep_ms

from arduino import start
from arduino_alvik import ArduinoAlvik


alvik = ArduinoAlvik()


def setup():
    alvik.begin()


def loop():
    L, CL, C, CR, R = alvik.get_distance()
    T = alvik.get_distance_top()
    B = alvik.get_distance_bottom()
    print(f'T: {T} | B: {B} | L: {L} | CL: {CL} | C: {C} | CR: {CR} | R: {R}')
    sleep_ms(100)


def cleanup():
    alvik.stop()


start(setup=setup, loop=loop, cleanup=cleanup)
