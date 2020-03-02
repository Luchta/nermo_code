# nermo_code (Gazebo Version)
code repository to control the gazebo version of the NeRmo robot. You can find the simulation files here: https://github.com/Luchta/nermo_simulation

For other Versions switch branch:
* master: Code to control the final version of NeRmo (V4.1)
* gazebo: Code to control the gazebo simulation of NeRmo
* version4: Code for version 4 of NeRmo (Japan Experiments)
* version2to3: Code for versions 2,2.1 and 3 of NeRmo
* version1: Code for the first version of NeRmo

## DISCLAIMER

 This submission is to be used within the general terms of service of the Human Brain Project (HBP). In particular, users should not download the following code if they intend to make a military use of it.

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
2. In a new terminal start gazebo
3. Load Nermo into Gazebo
4. In a new terminal start the publisher
5. you can now control the robot in gazebo