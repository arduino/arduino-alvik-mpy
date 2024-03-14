from sys import exit
from arduino_alvik.stm32_flash import *

if CHECK_STM32.value() is not 1:
    print("Turn on your Alvik to continue...")
    while CHECK_STM32.value() is not 1:
        sleep_ms(500)

ans = STM32_startCommunication()
if ans == STM32_NACK:
    print("Cannot establish connection with STM32")
    exit(-1)

print('\nSTM32 FOUND')

print('\nERASING MEM')
STM32_eraseMEM(0xFFFF)

print("\nWRITING MEM")
STM32_writeMEM("firmware.bin")
print("\nDONE")
print("\nLower Boot0 and reset STM32")

STM32_endCommunication()
