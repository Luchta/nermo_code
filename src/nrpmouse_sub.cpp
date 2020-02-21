#include "ros/ros.h"
//#include "nrpmouse_pkg/nrpmouse_msg.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include <stdio.h>
#include <stdlib.h>

//void clbk(const nrpmouse_pkg::nrpmouse_msg::ConstPtr& msg)
//void clbk(const std_msgs::Float64MultiArray::ConstPtr& msgarr)
void clbk(const std_msgs::Float64::ConstPtr& sensor)
//void clbk(const std_msgs::Float64MultiArray::ConstPtr& sensor)
//void clbk(const std_msgs::String::ConstPtr& sensor)
 {

    //ROS_INFO("%s", sensor->data.c_str());
    ROS_INFO("%f", sensor->data);
    /*
    
    ROS_INFO("%f", sensor->data[1]);
    ROS_INFO("%f", sensor->data[2]);
    ROS_INFO("%f", sensor->data[3]);
    ROS_INFO("%f", sensor->data[4]);
    ROS_INFO("%f", sensor->data[5]);
    ROS_INFO("%f", sensor->data[6]);
    ROS_INFO("%f", sensor->data[7]);
    ROS_INFO("%f", sensor->data[8]); 
    ROS_INFO("%f", sensor->data[9]); */

}

int main(int argc, char **argv)
{
  // ROS objects
  ros::init(argc, argv, "nrpmouse_sub");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("nrpmouse_sensortopic", 100, clbk);
  //std_msgs::Float64MultiArray sensor;
  //sensor.data.resize(10);
  ros::spin();

}