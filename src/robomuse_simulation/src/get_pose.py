#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import os
def callback(msg):
	var1 = round(msg.pose.pose.position.x, 2)
	var2 = round(msg.pose.pose.position.y, 2)
	var3 = round(msg.pose.pose.position.z, 2)
	print('x=',var1)
	print('y=',var2)
	print('z=',var3)
	return (msg)

rospy.init_node('get_pose')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()
