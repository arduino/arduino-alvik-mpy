#import demo

#import hand_follower.py

#import line_follower.py

#this demo will create a wifi network, and connecting to it with a phone will show on 192.168.4.1 a page with four buttons to move the robot
#you should change the name of the wifi network by editing wifi_host.py line 31 the variable gs_ssid with the name of the robot
#multiple robot with the same wifi network name would interfere with each other
import wifi_host.py
