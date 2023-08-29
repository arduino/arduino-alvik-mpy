from stm32_flash import *

ans = STM32_startCommunication()
if ans == STM32_NACK:
    print("Cannot etablish connection with STM32")
    exit(-1)

print('\nSTM32 FOUND')

print('\nSTM32 GET ID')
id_ = STM32_getID()
print(id_.hex())

print('\nSTM32 GET VERSION')
ver = STM32_getVER()
print(ver.hex())

print('\nSTM32 GET')
get_all = STM32_get()
print(get_all.hex())
