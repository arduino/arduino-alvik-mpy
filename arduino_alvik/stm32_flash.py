import os
from time import sleep_ms
from machine import UART, Pin

A6 = 13                                         # ESP32 pin13 -> nano A6/D23
CHECK_STM32 = Pin(A6, Pin.IN)    # nano A6/D23 -> TO CHECK STM32 IS ON

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
STM32_ERASE = b'\x44'   # 0x44 is Extended Erase for bootloader v3.0 and higher. 0x43 is standard (1-byte address) erase

STM32_ADDRESS = bytes.fromhex('08000000')  # [b'\x08',b'\x00',b'\x00',b'\x00']

# NANO ESP32 SETTINGS
_D2 = 5     # ESP32 pin5 -> nano D2
_D3 = 6     # ESP32 pin6 -> nano D3
_Boot0 = Pin(_D2, Pin.OUT)      # STM32 Boot0
_NRST = Pin(_D3, Pin.OUT)       # STM32 NRST

# UART SETTINGS
_UART_ID = 1
_TX_PIN = 43
_RX_PIN = 44
_BAUDRATE = 115200
_BITS = 8
_PARITY = 0
_STOP = 1

readAddress = bytearray(STM32_ADDRESS)
writeAddress = bytearray(STM32_ADDRESS)

uart = UART(_UART_ID, baudrate=_BAUDRATE, bits=_BITS, parity=_PARITY, stop=_STOP, tx=_TX_PIN,
            rx=_RX_PIN)  # parity 0 equals to Even, 1 to Odd


def STM32_startCommunication() -> bytes:
    """
    Starts communication with STM32 sending just 0x7F. Blocking
    :return:
    """
    STM32_bootMode(bootloader=True)
    STM32_reset()
    uart.write(STM32_INIT)
    return _STM32_waitForAnswer()


def STM32_endCommunication():
    """
    Ends communication with STM32 restoring flash boot mode and resetting
    :return:
    """
    STM32_bootMode(bootloader=False)
    STM32_reset()


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


def STM32_reset():
    """
    Resets STM32 from the host pins _D3
    :return:
    """
    _NRST.value(0)
    sleep_ms(100)
    _NRST.value(1)
    sleep_ms(500)


def STM32_bootMode(bootloader: bool = False):
    """
    Sets boot mode for STM32
    :param bootloader: if True, STM32 bootloader is run on boot
    :return:
    """
    _Boot0.value(bootloader)


def STM32_sendCommand(cmd: bytes):
    """
    Sends a command and its complement according to AN3155
    :param cmd: the command byte
    :return:
    """
    _cmd = bytes([cmd[0] ^ 0xFF])
    uart.write(cmd)
    uart.write(_cmd)


def STM32_readResponse() -> [bytearray, bytes]:
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
            return STM32_NACK
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
    if res == STM32_NACK:
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
    if res == STM32_NACK:
        print("GET VER: STM32 responded with NACK")
        return bytearray(0)
    return res


def _STM32_readMode() -> bytes:
    """
    Enters read memory mode. Blocking
    :return: returns ACK or NACK
    """
    STM32_sendCommand(STM32_READ)
    return _STM32_waitForAnswer()


def _STM32_writeMode() -> bytes:
    """
    Enters write memory mode. Blocking
    :return: returns ACK or NACK
    """
    STM32_sendCommand(STM32_WRITE)
    return _STM32_waitForAnswer()


def _STM32_eraseMode() -> bytes:
    """
    Enters erase memory mode. Blocking
    :return: returns ACK or NACK
    """
    STM32_sendCommand(STM32_ERASE)
    return _STM32_waitForAnswer()


def _STM32_sendAddress(address: bytes) -> bytes:
    """
    Sends the start address of read/write operations. Blocking
    :param address:
    :return:
    """
    assert len(address) == 4

    checksum = address[0] ^ address[1] ^ address[2] ^ address[3]
    uart.write(address)
    uart.write(bytes([checksum]))

    return _STM32_waitForAnswer()


def _incrementAddress(address: bytearray):
    """
    Increments address by one page (256 bytes)
    :param address:
    :return:
    """

    address[2] = address[2] + 1
    if address[2] == 0:
        address[1] = address[1] + 1
        if address[1] == 0:
            address[0] = address[0] + 1


def _STM32_readPage() -> bytearray:
    """
    Reads a 256 bytes data page from STM32. Returns a 256 bytearray. Blocking
    :return: page bytearray
    """

    STM32_sendCommand(b'\xFF')
    res = _STM32_waitForAnswer()
    if res != STM32_ACK:
        print("READ PAGE: Cannot read STM32")
        return bytearray(0)
    out = bytearray(0)
    i = 0
    while i < 256:
        b = uart.read(1)
        if b is None:
            continue
        out.append(b[0])
        i = i+1
    return out


def _STM32_flashPage(data: bytearray) -> bytes:
    """
    Sends a 256 bytes data page to STM32. Blocking
    :param data:
    :return:
    """

    assert len(data) == 256

    uart.write(b'\xff')     # page length
    checksum = 0xff         # starting checksum = page length

    for d in data:
        uart.write(bytes([d]))
        checksum = checksum ^ d

    uart.write(bytes([checksum]))

    return _STM32_waitForAnswer()


def STM32_readMEM(pages: int):
    """
    Reads n 256-bytes pages from memory
    :param pages: number of pages to read
    :return:
    """

    for i in range(0, pages):
        if _STM32_readMode() != STM32_ACK:
            print("COULD NOT ENTER READ MODE")
            return

        if _STM32_sendAddress(readAddress) != STM32_ACK:
            print("STM32 ERROR ON ADDRESS SENT")
            return

        page = _STM32_readPage()
        print(f"Page {i+1} content:\n")
        print(page.hex())

        _incrementAddress(readAddress)


def STM32_writeMEM(file_path: str, toggle: "Generator" = None):

    with open(file_path, 'rb') as f:
        print(f"Flashing {file_path}\n")
        file_size = os.stat(file_path)[-4]
        file_pages = int(file_size / 256) + (1 if file_size % 256 != 0 else 0)
        i = 1
        while True:
            data = bytearray(f.read(256))
            read_bytes = len(data)
            if read_bytes == 0:
                break
            data.extend(bytearray([255]*(256-read_bytes)))  # 0xFF padding

            if _STM32_writeMode() != STM32_ACK:
                print("COULD NOT ENTER WRITE MODE")
                return

            if _STM32_sendAddress(writeAddress) != STM32_ACK:
                print("STM32 ERROR ON ADDRESS SENT")
                return

            if _STM32_flashPage(data) != STM32_ACK:
                print(f"STM32 ERROR FLASHING PAGE: {writeAddress}")
                return

            percentage = int((i / file_pages) * 100)
            print("\033[2K\033[1G", end='\r')
            print(f"Flashing STM32: {percentage:>3}%", end='')
            i = i + 1
            _incrementAddress(writeAddress)
            if toggle is not None:
                next(toggle)


def _STM32_standardEraseMEM(pages: int, page_list: bytearray = None):
    """
    Standard Erase (0x43) flash mem pages according to AN3155
    :param pages: number of pages to be erased
    :param page_list: page codes to be erased
    :return:
    """

    if _STM32_eraseMode() == STM32_NACK:
        print("COULD NOT ENTER ERASE MODE")
        return

    if pages == 0xFF:
        # Mass erase
        uart.write(b'\xFF')
        uart.write(b'\x00')
    else:
        print("Not yet implemented erase")

    if _STM32_waitForAnswer() != STM32_ACK:
        print("ERASE OPERATION ABORTED")


def _STM32_extendedEraseMEM(pages: int, page_list: bytearray = None):
    """
    Extended Erase (0x44) flash mem pages according to AN3155
    :param pages: number of pages to be erased
    :param page_list: page codes to be erased
    :return:
    """

    if _STM32_eraseMode() == STM32_NACK:
        print("COULD NOT ENTER ERASE MODE")
        return

    if pages == 0xFFFF:
        # Mass erase
        uart.write(b'\xFF')
        uart.write(b'\xFF')
        uart.write(b'\x00')
    elif pages == 0xFFFE:
        # Bank1 erase
        uart.write(b'\xFF')
        uart.write(b'\xFE')
        uart.write(b'\x01')
    elif pages == 0xFFFD:
        # Bank2 erase
        uart.write(b'\xFF')
        uart.write(b'\xFD')
        uart.write(b'\x02')
    else:
        print("Not yet implemented erase")

    if _STM32_waitForAnswer() != STM32_ACK:
        print("ERASE OPERATION ABORTED")


def STM32_eraseMEM(pages: int, page_list: bytearray = None):
    """
    Erases flash mem pages according to AN3155
    :param pages: number of pages to be erased
    :param page_list: page codes to be erased
    :return:
    """

    if STM32_ERASE == b'\x43':
        _STM32_standardEraseMEM(pages, page_list)
    elif STM32_ERASE == b'\x44':
        _STM32_extendedEraseMEM(pages, page_list)
