# arduino-alvik-mpy

**Arduino Alvik micropython** library.



<br>
<br>



## How to Install Micropython library

### 1. install mpremote

[mpremote](https://docs.micropython.org/en/latest/reference/mpremote.html) is needed to upload files on the [Arduino® Nano ESP32](https://store.arduino.cc/products/nano-esp32?gad_source=1&gclid=Cj0KCQiA2KitBhCIARIsAPPMEhLtIxV_s7KyLJO4-69RdR1UeFTdgGK_XmI8w7xdbur4gs1oJU4Jl68aAhbaEALw_wcB).

```shell
(venv)$ pip install mpremote
```

### 2. install library

Run the following line to upload all files and download the dependencies needed to run the Arduino Alvik micropython library.

```shell
Linux
$ ./install.sh -p <device port>

Windows
> install.bat -p <device port>
```

__NOTE: DO NOT USE LAB FOR MICROPYTHON TO UPLOAD BIN FILES__

<br>
<br>

### 3. Update firmware on your Arduino® Alvik

Go into `utilities` folder and run:
```shell
Linux
$ ./flash_firmware.sh -p <device port> <path-to-your-firmware>

Windows
> flash_firmware.bat -p <device port> <path-to-your-firmware>
```
Answer `y` to flash firmware.

Note: The -p parameter is optional

<br>
<br>


## Examples

Use `mpremote` to copy files into your Arduino® Nano ESP32.

e.g.
``` shell
(venv)$ mpremote connect "COM1" fs cp ./examples/led_setting.py :led_setting.py
```

You can now use Arduino Lab for Micropython to run your examples remotely from the device filesystem.

<br>

__Note: not open bin files with Arduino Lab for Micropython because they will be corrupted__
