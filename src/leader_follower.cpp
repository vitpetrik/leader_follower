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
#include <algorithm>

#include "leader_follower.h"
#include <ros/ros.h>
#include <Eigen/Dense>

#include <math.h>

#define GOAL_DISTANCE 5.0

ros::Publisher force_pub;
ros::ServiceClient switch_ser;


double attractive_force(double distance, int order)
{
    if (distance < GOAL_DISTANCE)
        return 0;

    return (1/order)*pow(1/distance - 1/GOAL_DISTANCE , order);
}

double repulsive_force(double distance, int order)
{
    if (distance >= GOAL_DISTANCE)
        return 0;

    return (1/order)*pow(1/distance - 1/GOAL_DISTANCE , order);
}

void leader_cb(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
{
    mrs_msgs::SpeedTrackerCommand cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = msg.header.frame_id;

    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();

    for (auto element : msg.poses)
    {
        if(element.id != 0x0a)
            continue;
        Eigen::Vector3d body(element.pose.position.x, element.pose.position.y, element.pose.position.z);
        double distance = body.norm();
        body /= distance;

        ROS_INFO_STREAM("[LEADER_FOLLOWER] Distance: " << distance);

        double attractive_force = 0.01*pow(distance - GOAL_DISTANCE, 2);
        double repulsive_force = 0.01*(1 - 1/pow(distance/GOAL_DISTANCE, 2));

        ROS_INFO_STREAM("[LEADER_FOLLOWER] repulsive_force: " << repulsive_force);

        if (distance >= GOAL_DISTANCE)
            acceleration += attractive_force * body;
        else
            acceleration += repulsive_force * body;

    }

    ROS_INFO_STREAM("[LEADER_FOLLOWER] Force: " << acceleration.transpose());

    cmd.acceleration.x = acceleration(0);
    cmd.acceleration.y = acceleration(1);
    cmd.acceleration.z = 0;
    cmd.heading = atan2(acceleration(1), acceleration(0));
    cmd.use_heading = true;
    cmd.height = 5;
    cmd.use_acceleration = true;
    cmd.use_height = true;

    force_pub.publish(cmd);
    mrs_msgs::String tracker;
    tracker.request.value = "SpeedTracker";
    bool status = switch_ser.call(tracker);

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_follower");
    ros::NodeHandle nh("~");

    ros::Subscriber pose_sub = nh.subscribe("leader", 10, leader_cb);
    force_pub = nh.advertise<mrs_msgs::SpeedTrackerCommand>("force", 10);

    switch_ser = nh.serviceClient<mrs_msgs::String>("switch_tracker");

    // mrs_msgs::SpeedTrackerCommand cmd;
    // cmd.header.stamp = ros::Time::now();
    // cmd.header.frame_id = "fcu";
    // cmd.velocity.x = 0;
    // cmd.velocity.y = 0;
    // cmd.velocity.z = 0;
    // cmd.acceleration.x = 0;
    // cmd.acceleration.y = 0;
    // cmd.acceleration.z = 0;
    // cmd.use_acceleration = true;
    // cmd.use_velocity = true;

    // force_pub.publish(cmd);
    // mrs_msgs::String tracker;
    // tracker.request.value = "SpeedTracker";
    // bool status = switch_ser.call(tracker);

    // ROS_INFO("[LEADER_FOLLOWER] Switched to SpeedTracker: %d", status);

    ros::spin();
    return 0;
}