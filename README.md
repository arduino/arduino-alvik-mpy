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

Run `install.sh` (on Linux/MacOS) or `install.bat` (Windows) to upload all files needed to run the Arduino Alvik micropython library.

__NOTE: DO NOT USE LAB FOR MICROPYTHON TO UPLOAD BIN FILES__

<br>
<br>

## Examples

Use `mpremote` to copy files into your Arduino® Nano ESP32.

e.g.
``` shell
(venv)$ mpremote connect "COM1" fs cp ./examples/led_setting.py :led_setting.py
```

You can now use Arduino Lab for Micropython to run your examples remotely from the device filesystem.

__Note: not open bin files with Arduino Lab for Micropython because they will be corrupted__
