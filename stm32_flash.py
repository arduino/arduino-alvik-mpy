import time
from machine import Pin, UART


STM32_INIT = b'\x7F'
STM32_NACK = b'\x1F'
STM32_ACK = b'\x79'

STM32_GET = b'\x00'
STM32_ID = b'\xFD' #should be 0x02

STM32_READ = b'\x11'
STM32_RUN = b'\x21'
STM32_WRITE = b'\xCE' #0xCE 0x31

STM32_ADDRESS = bytes.fromhex('08000000') #[b'\x08',b'\x00',b'\x00',b'\x00']









led = Pin(48,Pin.OUT)
led_state = False

uart = UART(1, baudrate=115200, bits=8, parity=0 , stop=1, tx=43,rx=44) #parity 0 equals to Even, 1 to Odd



def STM32_sendCommand(x):
    value = x
    value_not = bytes([value[0]^0xFF])
    uart.write(value)
    uart.write(value_not)




def STM32_checkOK():
    result = -1
    reply = uart.read(1)
    if reply != None:
        if reply == STM32_ACK:
            result=0
        else:
            if reply == STM32_NACK:
                result=1
    return result
     
        

def STM32_waitOK():
    result=-1
    while result<0:
        result = STM32_checkOK()
    return result
      
      
        
def STM32_getID():
    lock = True
    while lock:
        STM32_sendCommand(STM32_ID)
        if STM32_waitOK()==0:
            lock = False
        
    lock = True
    while lock:
        data = uart.read(1)
        if data != None:
            N = data[0]
            lock = False
    stm32id = uart.read(N+1)
    STM32_waitOK()
    return stm32id
    
    
    
def STM32_writeMode():
    lock = True
    while lock:
        STM32_sendCommand(STM32_WRITE)
        if STM32_waitOK()==0:
            lock = False

        
        
def STM32_address(address):
    check = 0x00
    check = check^address[0]
    check = check^address[1]
    check = check^address[2]
    check = check^address[3]
    
    uart.write(address)
    uart.write(bytes([check]))



print('\nSTM32 FIRMWARE UPDATER')

STM32_sendCommand(STM32_INIT) 
STM32_waitOK()

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

with open('Blink_fast.bin','rb') as file_t:
    blob_data = bytearray(file_t.read())
    print(blob_data)


"""

x=int.from_bytes(STM32_ADDRESS,'big')
x=x+127
new_address=x.to_bytes(4,'big')
STM32_address(new_address)

STM32_waitOK()
"""




    