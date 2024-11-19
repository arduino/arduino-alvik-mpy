import sys


def reload_modules():
    to_be_reloaded = []

    for m in sys.modules:
        to_be_reloaded.append(m)
        del sys.modules[m]

    for m in to_be_reloaded:
        exec(f'import {m}')


reload_modules()
from arduino_alvik import update_firmware

# this is a patch to fix possible running threads on Alvik
from arduino_alvik import ArduinoAlvik
alvik = ArduinoAlvik()
alvik.stop()

update_firmware('/firmware.bin')
