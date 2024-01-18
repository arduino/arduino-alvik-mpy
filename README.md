# Install Micropython firmware

_First Install mpremote_

mpremote is needed to upload files on the nano-ESP32.

__DO NOT USE LAB FOR MICROPYTHON TO UPLOAD BIN FILES__

```shell
(venv)$ pip install mpremote
```

_Using scripts_

Please run install.sh (on Linux) or install.bat (Windows) to upload all the files needed to run the robot MP firmware

### Exmples

_Upload the examples you need with mpremote_

e.g.
``` shell
(venv)$ mpremote connect "COM1" fs cp ./examples/led_setting.py :led_setting.py
```

You can now use Arduino Lab for Micropython to run your examples remotely from the device filesystem
