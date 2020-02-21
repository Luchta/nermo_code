#include <iostream>
#include "cmouse_ctrl.h"

// ROS includes
#include "ros/ros.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "nrpmouse_pub");

    std::cout << "Hello World!" << std::endl;

    CMouseRos Mouse = CMouseRos();
    //CMouseCtrl Mouse = CMouseCtrl();
    CMouseUI UI = CMouseUI(Mouse.messages);

    Mouse.ROSstartThread();

    UI.process();
    //Mouse.RosCtrl();

    //ros::spin();

    return 0;
}

