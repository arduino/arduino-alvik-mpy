from time import sleep_ms
from machine import Pin, UART

STM32_INIT = b'\x7F'
STM32_NACK = b'\x1F'
STM32_ACK = b'\x79'

# STM32 COMMANDS
STM32_GET = b'\x00'
STM32_GET_VERSION = b'\x01'
STM32_GET_ID = b'\x02'
STM32_READ = b'\x11'
STM32_GO = b'\x21'
STM32_WRITE = b'\x31'
STM32_ERASE = b'\x43'

STM32_ADDRESS = bytes.fromhex('08000000')  # [b'\x08',b'\x00',b'\x00',b'\x00']

# UART SETTINGS
_UART_ID = 1
_TX_PIN = 43
_RX_PIN = 44
_BAUDRATE = 115200
_BITS = 8
_PARITY = 0
_STOP = 1

# led = Pin(48, Pin.OUT)
# led_state = False

uart = UART(_UART_ID, baudrate=_BAUDRATE, bits=_BITS, parity=_PARITY, stop=_STOP, tx=_TX_PIN,
            rx=_RX_PIN)  # parity 0 equals to Even, 1 to Odd


def STM32_startCommunication() -> bytes:
    """
    Starts communication with STM32 sending just 0x7F. Blocking
    :return:
    """
    uart.write(STM32_INIT)
    return _STM32_waitForAnswer()


def _STM32_waitForAnswer() -> bytes:
    """
    Blocking wait
    :return: returns ACK or NACK
    """

    while True:
        res = uart.read(1)
        if res == STM32_ACK or res == STM32_NACK:
            break
        else:
            sleep_ms(10)

    return res


def STM32_sendCommand(cmd: bytes):
    """
    Sends a command and its complement according to AN3155
    :param cmd: the command byte
    :return:
    """
    _cmd = bytes([cmd[0] ^ 0xFF])
    uart.write(cmd)
    uart.write(_cmd)


def STM32_readResponse() -> [bytearray, int]:
    """
    Blocking read to get the STM32 response to command, according to AN3155
    :return: returns a response bytearray dropping leading and trailing ACKs. returns -1 if NACK
    """
    out = bytearray(0)

    acks = 0
    while True:
        b = uart.read(1)
        if b is None:
            continue
        if b == STM32_NACK:
            return -1
        elif b == STM32_ACK:
            if acks == 1:
                break
            else:
                acks = acks + 1
                continue
        out.append(b[0])

    return out


def STM32_get() -> bytearray:
    """
    GET Command according to AN3155
    :return: returns a bytearray containing bootloader version (1 byte) and available commands
    """
    STM32_sendCommand(STM32_GET)
    res = STM32_readResponse()
    if res == -1:
        print("GET: STM32 responded with NACK")
        return bytearray(0)
    return res[1:]


def STM32_getID() -> bytearray:
    """
    GET ID Command according to AN3155
    :return: returns device ID (2 bytes)
    """
    STM32_sendCommand(STM32_GET_ID)
    res = STM32_readResponse()
    if res == -1:
        print("GET_ID: STM32 responded with NACK")
        return bytearray(0)
    return res[1:]


def STM32_getVER() -> bytearray:
    """
    GET ID Command according to AN3155
    :return: returns bootloader version (3 bytes)
    """
    STM32_sendCommand(STM32_GET_VERSION)
    res = STM32_readResponse()
    if res == -1:
        print("GET VER: STM32 responded with NACK")
        return bytearray(0)
    return res


def STM32_writeMode():
    lock = True
    while lock:
        STM32_sendCommand(STM32_WRITE)
        if STM32_waitOK() == 0:
            lock = False


def STM32_address(address):
    check = 0x00
    check = check ^ address[0]
    check = check ^ address[1]
    check = check ^ address[2]
    check = check ^ address[3]

    uart.write(address)
    uart.write(bytes([check]))
