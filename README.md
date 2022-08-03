
# Turtle Controller

This package provides basic locomotion behaviour e.g GoToPos, FollowTraj etc for differential drive robot. This package has two implementations, one in Gazebo turtlebot3, another in Zemo_rover (hardware implementation). Basic contoller node stays same for both implementations.



## Deployment


Clone the **zemo_rover** repository on the robot companion computer. Then build the package and launch 

```bash
roslaunch zemo_rover zemo_robot.launch
```
This will start "serial_node" for communicating with Arduino that subscribes to **"/cmd_vel"** topic, and "diff_tf_node" that computes the transformation(tf) and publishes the odometry data in **"/odom"** topic.

On work station system run:

```bash
roslaunch turtle_controller goToPos_Zemo.launch controller_reconfig:=true
```
The **"_Zemo"** tag in the launch file signifies that the launch file is related to hardware implementation. The **"controller_reconfig:=true"** starts the "rqt_reconfigure" to set controller gains dynamically.


 

## Required ROS Packages

For simulations **turtlebot3_gazebo** package.
For hardware implementaion in Zemo, we need **zemo_rover**, **differential-drive** and **rosserial** ROS packages.
Other than that for work station computer we need **rviz**, **teleop_twist_keyboard**.
## Connect the Robot with Work Station

Assuming that roscore will run in the work station system and the ip address:
Work_Station: 192.168.29.72
Robot_Computer: 192.168.29.159

First look for the available devices on the network:
```bash
nmap -sP 192.168.29.0/24
```
Identify the ip address of both the systems.
Then ssh to the robot computer form Work_Station and export the URI and set the ROS_IP:
```bash
ssh ubuntu@192.168.29.159
export ROS_MASTER_URI=http://192.168.29.72:11311
export ROS_IP=192.168.29.159
```
In Work_Station's terminal
```bash
export ROS_IP=192.168.29.72
```
