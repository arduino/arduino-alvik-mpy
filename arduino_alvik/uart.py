from machine import UART

# UART SETTINGS
_UART_ID = 1
_TX_PIN = 43
_RX_PIN = 44
_BAUDRATE = 460800
_BITS = 8
_PARITY = None
_STOP = 1

uart = UART(_UART_ID, baudrate=_BAUDRATE, bits=_BITS, parity=_PARITY, stop=_STOP, tx=_TX_PIN,
            rx=_RX_PIN)  # parity 0 equals to Even, 1 to Odd
