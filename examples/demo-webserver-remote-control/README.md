# arol-alvik-demos

Demos for [Arduino Alvik](https://www.arduino.cc/education/arduino-alvik/) developed by [AROL Closure Systems](https://www.arol.com/).

# Instructions

## 1 - Download IDE

[Download IDE](https://labs.arduino.cc/en/labs/micropython)

Arduino labs for Micropython

Unzip the compressed folder

Run the executable

[Either clone or download this repository as zip](https://github.com/arolgroup/arol-alvik-demos/archive/refs/heads/main.zip)

Unzip this repository

## 2 - Connect to the Alvik

Power on the Alvik with its switch

Connect to the Alvik with a USB C cable

![](/images/1-connect-usb-c.jpg)

## 3 - Transfer demo files to the Alvik

Click on the file icon

Click on the folder path on the right pane and navigate to the folder with the files in this repository

![](/images/3-transfer-files.JPG)

One at a time, select each of the following files and with the left arrow transfer them to the Alvik
- main.py : automatically executed by the robot at startup
- wifi_host.py : demo implementation of a wifi webserver hosted by the Alvik
- page_arrow_control.html : web page to be displayed in the device used to control the Alvik. Can be either a phone or a computer

## 4 - Rename the robot

Double click on "wifi_host.py" on the left pane (inside the robot) and edit the name of the wifi network to the desired name of your robot.

It should be a unique name as not to interfere with other Alvik in the same room or with other networks. I advics calling it "Alvik-myname" for ease

Save the file, it will take a few seconds. On the black console down it should show <OK> repeated a number of times. When it stops, save is complete.

![](/images/4-rename.JPG)

## 5 - run applications - reboot the robot

There is a button near the USB-C port on the alvik. Pressing it will reboot the robot. The robot will always play main.py when rebooted.

You can also run application by using the play button on an opened file in the IDE.

The black console will show the "print" strings when connected to the robot.

You will need to reconnect on the IDE.

## 6 - Connect to the robot via wifi

Wait a few seconds after reboot

In the list of wifi networks now your chosen robot name will show up in the list of networks

Connect to the wifi network, you should get a prompt for a sign in page a after a few seconds, or be redirected to the html page hosted by the robot

Pushing the arrow buttons will now move the robot, this does NOT require the USB-C cable plugged in

![](/images/5-connect-wifi-phone.jpg)

![](/images/5-connect-wifi-pc.JPG)

## 7 Enjoy the wifi remote controls wia webpage hosted by the Alvik! 

BUG: Samsung and Apple devices will not redirect automatically. To control the robot, open a web browser with wifi conencted to the Alvik, and go to the web page "192.168.4.1"

https://github.com/user-attachments/assets/10da174e-641d-4e57-8ab0-77a9fef9af31

## 8 - Stop the webserver

Clicking on the red button in the web page will stop the web server and show a disconnection page. The wifi will disconnect.

To restart the application you need to restart the robot, or need to play from the IDE.

This helps you in uploading new code to the robot without needing to reset, as it stops the wifi radio and threads properly.

## Copyright and license

Copyright (C) 2024-2025, [AROL Closure Systems](https://www.arol.com/), all rights reserved.

The source code contained in this repository and the executable distributions are licensed under the terms of the MIT license as detailed in the [LICENSE](LICENSE) file.

<!-- EOF -->
