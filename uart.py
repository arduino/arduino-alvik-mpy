import time
from machine import Pin, UART

led = Pin(48,Pin.OUT)
led_state = False

uart = UART(1,baudrate=9600,tx=43,rx=44)

uart.write('ciao')

start = time.ticks_ms()

while True:
    result = uart.read()
    if result!= None:
        print(result)
    delta = time.ticks_diff(time.ticks_ms(), start) 
    if delta>1000:
        start=time.ticks_ms()
        if led_state == False:
            led.on()
            led_state=True
        else:
            led.off()
            led_state=False
    