#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from tf.transformations import euler_from_quaternion

def get_rotation (msg:Odometry):
    global x,y,roll, pitch, yaw, pub, curr_pose
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    curr_pose.x = msg.pose.pose.position.x
    curr_pose.y = msg.pose.pose.position.y
    curr_pose.theta = yaw
    pub.publish(curr_pose)

rospy.init_node('odom_to_Pose2D')

sub = rospy.Subscriber ('/odom', Odometry, get_rotation)
pub = rospy.Publisher('/turtle_pose2d', Pose2D, queue_size=10)
curr_pose = Pose2D()

r = rospy.Rate(10)
while not rospy.is_shutdown():
    r.sleep()