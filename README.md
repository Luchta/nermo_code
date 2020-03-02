# nermo_code
code repository to control versions 2 to 3 of the NeRmo robot.

For other Versions switch branch:


## Prerequisites
setup a catkin workspace with ros:
```
mkdir -p ~/mouse_ws/src
cd ~/mouse_ws/
catkin_make
```

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
1. Start roscore
2. In a new terminal start the BT connection to the Mouse
```
sudo rfcomm release 0 20:16:03:08:38:19
```
This is to make sure the address is not assigned to any serial port
You only need to do this at the first time you setup the mouse
```
sudo rfcomm bind 0 20:16:03:08:38:19
```
This is to bind the address to rfcomm0, so that you know which port to connect to it via the
rosserial_python
You only need to do this at the first time you setup the mouse
```
rosrun rosserial_python serial_node.py /dev/rfcomm0 _baud:=115200
```
This initiates the ROS /serial_node. It is communicating via rfcomm0 as we bind it to rfcomm0 using
the sudo rfcomm bind 0

3.In a third terminal, start the publisher
```
cd mouse_ws/
source devel/setup.bash
rosrun nrp_mouse nrpmouse_pub
```
