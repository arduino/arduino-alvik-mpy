from sys import exit
from stm32_flash import *

ans = STM32_startCommunication()
if ans == STM32_NACK:
    print("Cannot etablish connection with STM32")
    exit(-1)

print('\nSTM32 FOUND')

print('\nERASING MEM')
STM32_eraseMEM(0xFFFF)

print("\nWRITING MEM")
STM32_writeMEM("Blink_fast.bin")
print("\nDONE")
print("\nLower Boot0 and reset STM32")

STM32_endCommunication()
