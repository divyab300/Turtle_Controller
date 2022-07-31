#ifndef GO2GOAL_H
#define GO2GOAL_H

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include "turtle_controller/NavTarget.h"

bool controller_busy;
float Kv = 0.5; // velocity gain
float Kh = 0.5; // heading gain
float x_vel_max = 0.5; // Maximum linear velocity
float w_vel_max = 4.0; // Maximum angular velocity

geometry_msgs::Pose2D goal_pose; // desired goal of the robot
geometry_msgs::Pose2D curr_pose; // current pose of the robot 
geometry_msgs::Twist curr_cmd;   // computed control command

ros::Publisher cmd_pub;      // Publishes to "/cmd_vel"
ros::Subscriber pose_sub;    // Subscribes to "turtle_pose2d"
ros::ServiceServer get_goal; // ROS service "/Service_SetGoal"

void updatePoseCB(const geometry_msgs::Pose2D& new_pose);
void compute_cmd(); // Computes command velocities
float constrain(float x, float limit);
bool set_target(turtle_controller::NavTarget::Request  &req, turtle_controller::NavTarget::Response &res);

#endif
