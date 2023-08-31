from sys import exit
from stm32_flash import *
from stm32_flash import _STM32_readMode, _STM32_sendAddress, _STM32_readPage

ans = STM32_startCommunication()
if ans == STM32_NACK:
    print("Cannot etablish connection with STM32")
    exit(-1)

print('\nSTM32 FOUND')

print('\nREADING MEMORY')

_STM32_readMode()
print('\nREAD MODE')

_STM32_sendAddress(STM32_ADDRESS)
print('\nADDRESS SENT')

page = _STM32_readPage()
print(page)
