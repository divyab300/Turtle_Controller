#include "ros/ros.h"
#include "turtle_controller/NavTarget.h"
#include "geometry_msgs/Twist.h"
#include "turtle_controller/go2goal.h"

bool set_target(turtle_controller::NavTarget::Request  &req, turtle_controller::NavTarget::Response &res)
{
  goal_pose = req.target;
  res.result.data = true;
  controller_busy = true;
  ROS_INFO("Received target: x=%f, y=%f", (float)goal_pose.x, (float)goal_pose.y);
  ROS_INFO("Sending back response: %d", (int)res.result.data);
  return true;
}

float constrain(float x, float limit)
{
  if (x>=limit)
    return limit;
  else if(x<limit && x>(-1*limit))
    return x;
  else if (x<=(-1*limit))
    return (-1*limit);
  else
    return 0;
}

void compute_cmd()
{
  float err_dist = sqrt((goal_pose.x-curr_pose.x)*(goal_pose.x-curr_pose.x)+(goal_pose.y-curr_pose.y)*(goal_pose.y-curr_pose.y));
  if(err_dist>0.05)
  {
    float des_theta = atan2((goal_pose.y-curr_pose.y),(goal_pose.x-curr_pose.x));
    float c = des_theta - curr_pose.theta;
    float err_theta = atan2(sin(c),cos(c));
    float x_vel = Kv*err_dist;
    float w_vel = Kh*err_theta;
    curr_cmd.linear.x = constrain(x_vel,x_vel_max);
    curr_cmd.angular.z = constrain(w_vel,w_vel_max);
    //std::cout<<"Des: "<<des_theta<<"Curr: "<<curr_pose.theta<<"Err: "<<err_theta<<std::endl;
  }
  else
  {
    controller_busy = false;
    curr_cmd.linear.x = 0;
    curr_cmd.angular.z = 0;
  }
}

void updatePoseCB(const geometry_msgs::Pose2D& new_pose)
{
  curr_pose = new_pose;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtle_control_server");
  ros::NodeHandle n;

  ros::Rate loop_rate(10);
  
  get_goal = n.advertiseService("Service_SetGoal", set_target);
  cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  pose_sub = n.subscribe("turtle_pose2d", 1000, updatePoseCB);
  ROS_INFO("Ready to receive target.");
  controller_busy = false; // initial setup 
  ROS_INFO("controller status: %d",controller_busy);
  
  while (ros::ok())
  {
    if(controller_busy)
    {
      compute_cmd();
      cmd_pub.publish(curr_cmd);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}