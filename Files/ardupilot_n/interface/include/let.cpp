#include <stdio.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
int main(int argc,char **argv){
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    std_msgs::String info;
    info.data="Starting..";
    ROS_INFO(info.data.c_str());
    int x=system("rosrun plane rt.py");

}