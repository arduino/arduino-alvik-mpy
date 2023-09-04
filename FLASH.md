# Flashing firmware on STM32 via UART

### How to

The steps to follow in order to flash a new firmware (.bin) file on STM32 are the following:

_Hardware_
* Connect STM32 _Boot0_ pin to VDD 3V3 or connect _Boot0_ to _D2_ pin (nano)
* Press _Reset_ button on Nucleo board or connect STM32 Reset pin to _D3_ (nano)
* Connect nano _TX_ pin to STM32 _D2_ (UART1 RX) pin
* Connect nano _RX_ pin to STM32 _D8_ (UART1 TX) pin
* Connect nano _GND_ to STM32 _GND_

_Software_
* Use `STM32_startCommunication` to connect to STM32
* Perform a memory erase (eg. mass extended erase `STM32_eraseMEM(0xFFFF)`)
* Write your .bin file: `STM32_writeMEM("Blink_fast.bin")`

If _Boot0_ is wired to 3V3, disconnect it and press STM32 _Reset_ pin

### Tested on

* STM32-F401RE