/**
 * @file leader_follower.cpp
 * @author Vit Petrik (petrivi2@fel.cvut.cz)
 * @brief Leader follower implementation using artificial potential field
 * @version 0.1
 * @date 2023-04-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "leader_follower.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_follower");
    ros::NodeHandle nh;

    ros::spin();
    return 0;
}