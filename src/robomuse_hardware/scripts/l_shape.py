#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32


class autonomous():
	def __init__(self):
		rospy.init_node("box_shape")
		nodename = rospy.get_name()
		rospy.loginfo("node %s started." % nodename)
		# subscribers
		rospy.Subscriber('filteredTheta', Float32, self.theta_callback)
		# publishers
		self.filter = rospy.Publisher('autonomous_cmd_vel', Twist, queue_size = 20)
		# variables
		self.rate = rospy.get_param("~rate", 20)
		# values for the filter
		self.yawAngle = 0.0
	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.spinOnce()
			r.sleep()
	def spinOnce(self):
		self.filter.publish(self.filteredTheta)
	# callback functions
	def theta_callback(self, msg):
		self.yawAngle = msg.data
		sC.boxPath

	def boxPath(self):
		print("Let's move your robot")
    		speed = input("Input your speed:")
                distance = input("Type your distance:")
                loops = input("number of loops?") # how many loops do you want 
		
	 	

if __name__ == '__main__':
	try:
		sC = autonomous()
		sC.spin()
	except rospy.ROSInterruptException:pass
