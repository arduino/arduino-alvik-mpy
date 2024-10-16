from arduino_alvik import ArduinoAlvik
from time import sleep_ms
import sys

alvik = ArduinoAlvik()

alvik.begin()

while True:
    try:
        print(f'\nLIBRARY VERSION: {alvik.get_lib_version()}')
        print(f'FIRMWARE VERSION: {alvik.get_fw_version()}')
        print(f'REQUIRED FW VERSION: {alvik.get_required_fw_version()}')
        print(f'FIRMWARE VERSION COMPATIBILITY CHECK: {alvik.check_firmware_compatibility()}\n')
        sleep_ms(1000)
    except KeyboardInterrupt as e:
        print('over')
        alvik.stop()
        sys.exit()