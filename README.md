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

***Note: The -p parameter is optional***


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

Use `mpremote` to copy following files from examples:
- `main.py`, the file which allows you to automatically start the demo
- `demo.py`, demo launcher
- `touch_move.py`, programming the robot movements via touch pads demo
- `line_follower.py`, black line follower demo
- `hand_follower.py`, hand following demo, the robot stays always at 10cm from an obstacle placed in front of it.

When the robot is turned on, the demo launcher starts after Alvik boot.

Blue leds on top turns on.

By pressing up and down arrows, it is possible to select different demos recognized by different distinct colors (blue, green and red).

Each color allows to run a different demo as following:
- `red` launchs touch move
- `green` launchs hand following
- `blue` launchs line follower.

To run a demo, `ok touch pad` pressing is needed after selected the right color demo.

To run a different demo, turn off and on the robot or reset the Arduino® Nano ESP32.

### 1. Touch mode example (RED)
This example starts with red leds on.

`directional touch pads` (up, down, left, right) program desired movements.

Everytime a directional touch pads is pressed, a violet blink happens on leds.

To clear the queue of commands, press the `cancel touch pad`.
A red blink happens on top leds.

To start the sequence, press the `ok touch pad`.

Everytime is possible to press `cancel touch pad` to stop the robot and reset the sequence.

<br>

### 2. Hand follower example (GREEN)
This example starts with green leds on.

Place an obstacle or the hand in front of the robot.

To start the robot press the `ok touch pad`.

The robot automatically moves itself to maintain 10 centimeters to the obstacle/hand.

It is possible to stop the robot everytime by pressing the `cancel touch pad`.

<br>

### 3. Line Follower example (BLUE)
This example starts with blue leds on.

To run this example, a white board and black tape (2cm wide) is required.

Place the robot at the center of the line and press `ok touch pad`.

It is possible to stop the robot everytime by pressing the `cancel touch pad`.










<br>
<br>
<br>

__Note: not open bin files with Arduino Lab for Micropython because they will be corrupted__
