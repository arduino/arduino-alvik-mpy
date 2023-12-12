# Flashing firmware on STM32 via UART

### How to

The steps to follow in order to flash a new firmware (.bin) file on STM32 are the following:

_Uploading the files_
Please install mpremote to upload files on nano-ESP32 especially for binaries

```shell
(venv)$ pip install mpremote
```
to upload a file:
``` shell
(venv)$ mpremote connect "COM1" fs cp firmware.bin :firmware.bin
```

_Procedure_
* Build your .bin firmware (e.g. in Arduino IDE select Sketch &#8594; Export Compiled Binary)
* Upload the utility/firmware_updater.py file
* Upload your .bin firmware file (e.g.)
* Perform a memory erase (eg. mass extended erase `STM32_eraseMEM(0xFFFF)`)
* In firmware_updater.py change the following line as needed:
``` python
STM32_writeMEM("firmware.bin")
```
* Run firmware_updater.py on your device

### Tested on

* nano-ESP32 STM32-F401RE
* nano-ESP32 STM32-F411RC