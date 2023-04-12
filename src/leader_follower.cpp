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

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/String.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "leader_follower.h"
#include <ros/ros.h>
#include <Eigen/Dense>

#include <math.h>

#define GOAL_DISTANCE 5.0

ros::Publisher force_pub;

void leader_cb(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
{
    mrs_msgs::SpeedTrackerCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = msg.header.frame_id;

    Eigen::Vector3d force = Eigen::Vector3d::Zero();

    for (auto element : msg.poses)
    {
        Eigen::Vector3d body(element.pose.position.x, element.pose.position.y, element.pose.position.z);
        double distance = body.norm();
        body /= distance;

        double attractive_force = 5*pow(distance - GOAL_DISTANCE, 2);
        double repulsive_force = -50*(1 - 1/pow(distance/GOAL_DISTANCE, 2));

        if (distance >= GOAL_DISTANCE)
            force += attractive_force * body;
        else
            force += repulsive_force * body;
    }

    cmd.force.x = force(0);
    cmd.force.y = force(1);
    cmd.force.z = force(2);
    cmd.use_force = true;

    force_pub.publish(cmd);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_follower");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("~leader", 10, leader_cb);
    force_pub = nh.advertise<mrs_msgs::PoseWithCovarianceArrayStamped>("~force", 10);

    ros::ServiceClient client = nh.serviceClient<mrs_msgs::String>("~switch_tracker");

    mrs_msgs::SpeedTrackerCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "fcu";
    cmd.velocity.x = 0;
    cmd.velocity.y = 0;
    cmd.velocity.z = 0;
    cmd.force.x = 0;
    cmd.force.y = 0;
    cmd.force.z = 0;
    cmd.use_force = true;
    cmd.use_velocity = false;

    force_pub.publish(cmd);
    mrs_msgs::String tracker;
    tracker.request.value = "SpeedTracker";
    bool status = client.call(tracker);

    ROS_INFO("[LEADER_FOLLOWER] Switched to SpeedTracker: %d", status);

    ros::spin();
    return 0;
}