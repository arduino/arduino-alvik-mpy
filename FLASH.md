# Flashing firmware on STM32 via UART

### How to

In order to flash a new firmware.bin file on STM32 follow these steps:

_Install mpremote_

mpremote is needed to upload files on the nano-ESP32.

__DO NOT USE LAB FOR MICROPYTHON TO UPLOAD BIN FILES__

```shell
(venv)$ pip install mpremote
```

_Procedure using scripts_
* Build your .bin firmware (e.g. in Arduino IDE select Sketch &#8594; Export Compiled Binary)
* Move your firmware binary to the arduino_robot_micropython/utilities folder
* Run flash_firmware.sh (Linux) or flash_firmware.bat (Windows) to upload the files and flash the binary passing the filename as a parameter:

``` shell
Linux
$ ./flash_firmware.sh -p <device_port> <my_firmware.bin>

Windows
> flash_firmware.bat -p <device_port> <my_firmware.bin>
```

Note: The -p parameter is optional

_Manual Procedure_
* Build your .bin firmware (e.g. in Arduino IDE select Sketch &#8594; Export Compiled Binary)
* Upload the utility/firmware_updater.py file
* Upload your .bin firmware file (e.g.)
* Perform a memory erase (eg. mass extended erase `STM32_eraseMEM(0xFFFF)`)
* In firmware_updater.py change the following line as needed:
``` python
STM32_writeMEM("firmware.bin")
```
* Run firmware_updater.py on your device

_Uploading files with mpremote_

to upload a file:
``` shell
(venv)$ mpremote connect "COM1" fs cp firmware.bin :firmware.bin
```

### Tested on

* nano-ESP32 STM32-F401RE
* nano-ESP32 STM32-F411RC