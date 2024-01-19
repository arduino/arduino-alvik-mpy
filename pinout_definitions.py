from machine import Pin

# NANO to STM32 PINS
D2 = 5                              # ESP32 pin5 -> nano D2
D3 = 6                              # ESP32 pin6 -> nano D3
BOOT0_STM32 = Pin(D2, Pin.OUT)      # nano D2 -> STM32 Boot0
RESET_STM32 = Pin(D3, Pin.OUT)      # nano D2 -> STM32 NRST
