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
#if __INTELLISENSE__
#undef __ARM_NEON
#undef __ARM_NEON__
#endif

#include <mrs_msgs/PoseWithCovarianceArrayStamped.h>
#include <mrs_msgs/SpeedTrackerCommand.h>
#include <mrs_msgs/ReferenceStampedSrv.h>
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
ros::ServiceClient reference_ser;

#if 0

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

#endif

void leader_cb(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
{
    mrs_msgs::ReferenceStampedSrv ref;
    ref.request.header.stamp = ros::Time::now();
    ref.request.header.frame_id = msg.header.frame_id;

    Eigen::Vector3d new_position = Eigen::Vector3d::Zero();
    double heading;

    for (auto element : msg.poses)
    {
        Eigen::Quaterniond q(element.pose.orientation.w, element.pose.orientation.x, element.pose.orientation.y, element.pose.orientation.z);
        Eigen::Vector3d body(element.pose.position.x, element.pose.position.y, element.pose.position.z);

        new_position << -1, 0, 0;
        new_position = q*new_position;
        new_position(2) = 0;
        new_position.normalize();
        new_position *= GOAL_DISTANCE;

        new_position += body;

        heading = atan2(body(1), body(0));
    }

    ROS_INFO_STREAM("[LEADER_FOLLOWER] Force: " << new_position.transpose());
    ROS_INFO_STREAM("[LEADER_FOLLOWER] Heading: " << heading);

    ref.request.reference.position.x = new_position(0);
    ref.request.reference.position.y = new_position(1);
    ref.request.reference.position.z = new_position(2);
    ref.request.reference.heading = heading;

    reference_ser.call(ref);

    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_follower");
    ros::NodeHandle nh("~");

    ros::Subscriber pose_sub = nh.subscribe("leader", 10, leader_cb);
    force_pub = nh.advertise<mrs_msgs::SpeedTrackerCommand>("force", 10);
    switch_ser = nh.serviceClient<mrs_msgs::String>("switch_tracker");
    reference_ser = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("reference");

    ros::spin();
    return 0;
}