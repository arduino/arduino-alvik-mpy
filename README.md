# arduino-alvik-mpy

**Arduino Alvik micropython** library.



<br>
<br>



## How to Install the Micropython library

### 1. install mpremote

[mpremote](https://docs.micropython.org/en/latest/reference/mpremote.html) is needed to upload files on the [Arduino® Nano ESP32](https://store.arduino.cc/products/nano-esp32?gad_source=1&gclid=Cj0KCQiA2KitBhCIARIsAPPMEhLtIxV_s7KyLJO4-69RdR1UeFTdgGK_XmI8w7xdbur4gs1oJU4Jl68aAhbaEALw_wcB).
Minimum suggested mpremote release is 1.22.0

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

***Note: The -p parameter is optional***


__WARNING: do not open bin files with Arduino Lab for Micropython 0.8.0 because they will be corrupted__

### 2.1  mip (MicroPython Package Manager)
This is the recommended method for boards which can connect to Internet. Make sure your board is connected to the Internet and
run the following MicroPython script using your favourite editor:

```py
import mip

mip.install('github:arduino/arduino-alvik-mpy')

```

<br>
<br>

### 3. Update firmware on your Arduino® Alvik

Download the latest [Arduino Alvik Carrier Firmware code](https://github.com/arduino-libraries/Arduino_AlvikCarrier) (to compile the firmware using Arduino IDE) or the [pre-compiled firmware](https://github.com/arduino-libraries/Arduino_AlvikCarrier/releases/latest)

Go into `utilities` folder and run:
```shell
Linux
$ ./flash_firmware.sh -p <device port> <path-to-your-firmware>

Windows
> flash_firmware.bat -p <device port> <path-to-your-firmware>
```
Answer `y` to flash firmware.

***Note: The -p parameter is optional***

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

## Default Demo

Use `mpremote` to copy following the files from the `examples\demo` folder:
- `main.py`, this file allows you to automatically start the demo
- `demo.py`, demo launcher
- `touch_move.py`, programming the robot movements via touch buttons demo
- `line_follower.py`, black line follower demo
- `hand_follower.py`, hand following demo, the robot stays always at 10cm from an obstacle placed in front of it.

When the robot is turned on, the demo launcher starts after Alvik boot.

Blue leds on top turn on.

By pressing up and down arrows, it is possible to select different demos identified by different colors (blue, green and red).

Each color allows to run a different demo as following:
- `red` launches the touch-move demo
- `green` launches the hand following demo
- `blue` launches the line follower demo

To run a demo, press the `OK touch button`, after selecting the right demo color.

To run a different demo, press the `CANCEL touch button` and you will be redirected to the main menu.

### 1. Touch mode example (RED)
This example starts with the red leds on.

`directional touch buttons` (UP, DOWN, LEFT, RIGHT) program the desired movements.

Everytime a `directional touch button` is pressed, the leds blink in a purple color indicating that the command has been registered.
- `UP touch button` will register a 10 cm forward movement
- `DOWN touch button` will register a 10 cm backward movement
- `RIGHT touch button` will register a 90° clockwise rotation movement
- `LEFT touch button` will register a 90° counterclockwise rotation movement

To clear the commands queue, press the `CANCEL touch button`.
The leds will blink in red.

To start the sequence, press the `OK touch button`.

Pressing the `CANCEL touch button` at any time stops the robot and resets the sequence.
Pressing the `CANCEL touch button` two times during sequence programming you will be redirected to the main menu.

<br>

### 2. Hand follower example (GREEN)
This example starts with the green leds on.

Place an obstacle or your hand in front of the robot.

To start the robot press the `OK touch button`.

The robot will move to keep a 10 centimeters distance from the obstacle/hand.

It is possible to stop the robot at any time by pressing the `CANCEL touch button`.

<br>

### 3. Line Follower example (BLUE)
This example starts with the blue leds on.

To run this example, a white board and black tape (2cm wide) is required.

Place the robot at the center of the line and press the `OK touch button`.

It is possible to stop the robot at any time by pressing the `CANCEL touch button`.



<br>
<br>
<br>

__WARNING: do not open bin files with Arduino Lab for Micropython 0.8.0 because they will be corrupted__


<br>
<br>
<br>


## Useful links
- [Arduino_Alvik](https://github.com/arduino-libraries/Arduino_Alvik): Arduino library required to program Alvik
- [Arduino_AlvikCarrier](https://github.com/arduino-libraries/Arduino_AlvikCarrier): Arduino library required to build the firmware
- [Arduino Alvik product page](https://store.arduino.cc/pages/alvik)
