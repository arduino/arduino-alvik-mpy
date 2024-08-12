from machine import Pin

# NANO to STM32 PINS
D2 = 5                                          # ESP32 pin5 -> nano D2
D3 = 6                                          # ESP32 pin6 -> nano D3
D4 = 7                                          # ESP32 pin7 -> nano D4

A4 = 11                                         # ESP32 pin11 SDA -> nano A4
A5 = 12                                         # ESP32 pin12 SCL -> nano A5
A6 = 13                                         # ESP32 pin13 -> nano A6/D23

BOOT0_STM32 = Pin(D2, Pin.OUT)                  # nano D2 -> STM32 Boot0
RESET_STM32 = Pin(D3, Pin.OUT)                  # nano D3 -> STM32 NRST
NANO_CHK = Pin(D4, Pin.OUT)                     # nano D4 -> STM32 NANO_CHK
CHECK_STM32 = Pin(A6, Pin.IN, Pin.PULL_DOWN)    # nano A6/D23 -> STM32 ROBOT_CHK
# ESP32_SDA = Pin(A4, Pin.OUT)                    # ESP32_SDA
# ESP32_SCL = Pin(A5, Pin.OUT)                    # ESP32_SCL

# LEDS
LEDR = Pin(46, Pin.OUT)                      #RED ESP32 LEDR
LEDG = Pin(0, Pin.OUT)                       #GREEN ESP32 LEDG
LEDB = Pin(45, Pin.OUT)                      #BLUE ESP32 LEDB
