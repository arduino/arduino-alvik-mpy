from stm32_flash import *

print('\nSTM32 FIRMWARE UPDATER')

STM32_startCommunication()
print('\nSTM32 FOUND')

print('\nSTM32 GETTING ID')
print(STM32_getID().hex())

STM32_writeMode()
print('\nSTM32 WRITE MODE')

"""
file = open('Blink_fast.bin','rb')
firmware = file.read()
firmware_size_256 = len(firmware)/256
firmware_size_256_last = len(firmware)%256
print(firmware_size_256)
print(firmware_size_256_last)
"""

with open('Blink_fast.bin', 'rb') as file_t:
    blob_data = bytearray(file_t.read())
    print(blob_data)

"""

x=int.from_bytes(STM32_ADDRESS,'big')
x=x+127
new_address=x.to_bytes(4,'big')
STM32_address(new_address)

STM32_waitOK()
"""
