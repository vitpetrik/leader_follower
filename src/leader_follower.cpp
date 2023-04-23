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
#include <mrs_msgs/PathSrv.h>
#include <mrs_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <mrs_lib/param_loader.h>

#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>

#include <algorithm>

#include "leader_follower.h"
#include <ros/ros.h>
#include <Eigen/Dense>
#include <tf/tf.h>

#include <math.h>

#define GOAL_DISTANCE 7.5

ros::Publisher force_pub;
ros::ServiceClient switch_ser;
ros::ServiceClient reference_ser;
ros::ServiceClient path_ser;

bool running = false;

int leader_id = 0;
double distance = 8;
double angle = M_PI;

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

double angdiff(double th1, double th2)
{
    double d = 0;
    d = th1 - th2;
    d = std::fmod(d + M_PI, 2 * M_PI) - M_PI;
    return d;
}

void leader_cb(const mrs_msgs::PoseWithCovarianceArrayStamped &msg)
{
    if (!running)
        return;

    mrs_msgs::ReferenceStampedSrv ref;
    ref.request.header.stamp = ros::Time::now();
    ref.request.header.frame_id = msg.header.frame_id;

    mrs_msgs::PathSrv path;
    path.request.path.header.stamp = ros::Time::now();
    path.request.path.header.frame_id = msg.header.frame_id;
    path.request.path.use_heading = true;
    path.request.path.fly_now = true;
    path.request.path.stop_at_waypoints = false;
    path.request.path.loop = false;
    path.request.path.override_constraints = false;
    path.request.path.relax_heading = false;

    Eigen::Vector3d target_pos = Eigen::Vector3d::Zero();
    double heading = 0;

    for (auto element : msg.poses)
    {
        if (element.id != leader_id)
            continue;

        Eigen::Quaterniond leader_q(element.pose.orientation.w, element.pose.orientation.x, element.pose.orientation.y, element.pose.orientation.z);
        Eigen::Vector3d leader_pos(element.pose.position.x, element.pose.position.y, element.pose.position.z);

        Eigen::Quaterniond target_q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                      Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                      Eigen::AngleAxisd(M_PI * angle / 180, Eigen::Vector3d::UnitZ());

        tf::Quaternion q_tf(element.pose.orientation.x,
                            element.pose.orientation.y,
                            element.pose.orientation.z,
                            element.pose.orientation.w);
        heading = atan2(leader_pos(1), leader_pos(0));
        // heading = tf::getYaw(q_tf);

        Eigen::Vector3d follower = leader_q.inverse() * (-leader_pos);
        follower(2) = 0;

        target_pos << distance, 0, 0;
        target_pos = target_q * target_pos;

        Eigen::Vector3d follower_tp = follower - target_pos;
        follower_tp = target_q.inverse() * follower_tp;

        if (follower_tp(0) >= -1.0 or follower_tp.norm() < 1)
        {
            Eigen::Vector3d new_position = leader_q * target_pos;
            new_position(2) = 0;

            new_position.normalize();
            new_position *= distance;

            new_position += leader_pos;

            mrs_msgs::Reference point;
            point.position.x = new_position(0);
            point.position.y = new_position(1);
            point.position.z = new_position(2);

            point.heading = atan2(new_position(1) - leader_pos(1), new_position(0) - leader_pos(0)) + M_PI;

            path.request.path.points.push_back(point);
        }
        else if (follower.norm() < distance + 0.5)
        {
            double current_angle = atan2(follower(1), follower(0));
            double final_angle = M_PI * angle / 180;
            double angle_increment = angdiff(final_angle, current_angle) / 20;

            for (int i = 1; i <= 20; i++)
            {
                Eigen::Quaterniond q_circle = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                              Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                              Eigen::AngleAxisd(current_angle + i * angle_increment, Eigen::Vector3d::UnitZ());

                Eigen::Vector3d circle_pos = Eigen::Vector3d::Zero();
                circle_pos << 1, 0, 0;
                circle_pos = q_circle * circle_pos;

                Eigen::Vector3d new_position = leader_q * circle_pos;

                new_position(2) = 0;
                new_position.normalize();
                new_position *= distance;

                heading = atan2(new_position(1), new_position(0)) + M_PI;

                new_position += leader_pos;

                if ((leader_pos - new_position).norm() < distance - 0.1)
                {
                    ROS_INFO_STREAM("[LEADER FOLLOWER] Something really cursed happend");
                    return;
                }

                mrs_msgs::Reference point;
                point.position.x = new_position(0);
                point.position.y = new_position(1);
                point.position.z = new_position(2);

                point.heading = heading;

                path.request.path.points.push_back(point);
            }
        }
        else
        {
            double final_angle = M_PI * angle / 180;

            double theta = acos(distance / follower.norm());
            double beta = atan2(follower(1), follower(0));

            double d1 = angdiff(-(theta + beta), 0);
            double d2 = angdiff(-(theta - beta), 0);

            double diff1 = angdiff(final_angle, d1);
            double diff2 = angdiff(final_angle, d2);

            double preffered_angle = 0;

            if (abs(diff1) <= abs(diff2))
                preffered_angle = d1;
            else
                preffered_angle = d2;

            Eigen::Quaterniond q_circle = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *
                                          Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                                          Eigen::AngleAxisd(preffered_angle, Eigen::Vector3d::UnitZ());

            Eigen::Vector3d circle_pos = Eigen::Vector3d::Zero();
            circle_pos << 1, 0, 0;
            circle_pos = q_circle * circle_pos;

            Eigen::Vector3d new_position = leader_q * circle_pos;

            new_position(2) = 0;
            new_position.normalize();
            new_position *= distance;

            new_position += leader_pos;

            Eigen::Vector3d line_increment = (new_position) / 10;

            for (int i = 0; i <= 10; i++)
            {
                Eigen::Vector3d new_position = line_increment * i;

                if ((leader_pos - new_position).norm() < distance - 0.1)
                {
                    ROS_INFO_STREAM("[LEADER FOLLOWER] Something really cursed happend");
                    return;
                }
                heading = atan2(leader_pos(1) - new_position(1), leader_pos(0) - new_position(0));

                mrs_msgs::Reference point;
                point.position.x = new_position(0);
                point.position.y = new_position(1);
                point.position.z = new_position(2);

                point.heading = heading;

                path.request.path.points.push_back(point);
            }
        }

        // sanity check

        path_ser.call(path);
        // running = false;
    }

    // ROS_INFO_STREAM_THROTTLE(1.0, "[LEADER_FOLLOWER] Force: " << new_position.transpose());
    // ROS_INFO_STREAM_THROTTLE(1.0, "[LEADER_FOLLOWER] Heading: " << heading);

    return;
}

bool stop(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    running = false;
    return true;
}

bool start(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    running = true;
    return true;
}

bool goto_position(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    running = true;
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "leader_follower");
    ros::NodeHandle nh("~");

    mrs_lib::ParamLoader param_loader(nh, "Leader follower");

    param_loader.loadParam("leader_id", leader_id);
    param_loader.loadParam("distance", distance);
    param_loader.loadParam("angle", angle);

    ros::ServiceServer start_follower = nh.advertiseService("start", start);
    ros::ServiceServer stop_follower = nh.advertiseService("stop", stop);
    ros::ServiceServer goto_position_follower = nh.advertiseService("goto_position", goto_position);

    ros::Subscriber pose_sub = nh.subscribe("leader", 10, leader_cb);
    force_pub = nh.advertise<mrs_msgs::SpeedTrackerCommand>("force", 10);
    switch_ser = nh.serviceClient<mrs_msgs::String>("switch_tracker");
    reference_ser = nh.serviceClient<mrs_msgs::ReferenceStampedSrv>("reference");
    path_ser = nh.serviceClient<mrs_msgs::PathSrv>("trajectory_path");

    running = false;

    ros::spin();
    return 0;
}