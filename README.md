# nermo_code
code repository to control the standard servo version 4 of the NeRmo robot.

For other Versions switch branch:
* master: Code to control the final version of NeRmo (V4.1)
* gazebo: Code to control the gazebo simulation of NeRmo
* version4: Code for version 4 of NeRmo (Japan Experiments)
* version2to3: Code for versions 2,2.1 and 3 of NeRmo
* version1: Code for the first version of NeRmo

## DISCLAIMER

 This submission is to be used within the general terms of service of the Human Brain Project (HBP). In particular, users should not download the following code if they intend to make a military use of it.

## Prerequisites

Setup a ROS environment on the RaspberryPi

Install the adafruit ServoKit Library

  https://learn.adafruit.com/adafruit-16-channel-pwm-servo-hat-for-raspberry-pi/using-the-python-library
  

## Setup & Compile
clone this repository into your

```
~/mouse_ws/src
```

in mouse_ws call:

```
catkin_make
```

## Run
1. start Roscore
2. start Publisher Node: 
  ```
   roslaunch bkp_mouse bkp_mouse_node
  ```
3. Run subscriber Python3 script
  ```
  adafruit_sub.py
  ```
##Control
control the robot via the keyboard:
i: inital pose
w: forward trott
a: left
d: right
q: close programm

