# nermo_code
 code repository to control all versions of the NeRmo robot, including the Gazebo simulation model

## DISCLAIMER

 This submission is to be used within the general terms of service of the Human Brain Project (HBP). In particular, users should not download the following code if they intend to make a military use of it.

## Usage

Choose your branch:
* master: Code to control the final version of NeRmo (V4.1)
* gazebo: Code to control the gazebo simulation of NeRmo
* version4: Code for version 4 of NeRmo (Japan Experiments)
* version2to3: Code for versions 2,2.1 and 3 of NeRmo
* version1: Code for the first version of NeRmo

## Setup
1. Clone this repository to the raspberry pi on the robot
2. in a terminal in this folder run in order
```
mkdir build
cd build
.. catkin_make
make
```
3. you can start the control programm using
```
.\Simple_Mouse
```
## License
[GPLv3](https://fsf.org/)
