from sys import exit
from stm32_flash import *

ans = STM32_startCommunication()
if ans == STM32_NACK:
    print("Cannot etablish connection with STM32")
    exit(-1)

print('\nSTM32 FOUND')

print("\nREADING 10 MEM PAGES")
STM32_readMEM(10)
