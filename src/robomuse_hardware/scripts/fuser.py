#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
class fuser():
	def __init__(self):
		rospy.init_node("fuser")
		nodename = rospy.get_name()
		rospy.loginfo("node %s started." % nodename)
		# subscribers
		rospy.Subscriber('yprEuler', Vector3, self.imu_callback)
		rospy.Subscriber('normalTheta', Float32, self.theta_callback)
		rospy.Subscriber('velo_centre', Float32, self.velocity_callback)
		# publishers
		self.filter = rospy.Publisher('totalYaw', Float32, queue_size = 20)
		# variables
		self.rate = rospy.get_param("~rate", 20)
		# values for the filter
		self.velocity = 0.0
		self.originalTheta = 0.0
		self.total_IMU_yaw = 0.0
		self.filteredTheta = 0.0
	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.spinOnce()
			r.sleep()
	def spinOnce(self):
		self.filter.publish(self.filteredTheta)
	# callback functions
	def imu_callback(self, msg):
		self.total_IMU_yaw = msg.x

	def theta_callback(self, msg):
		self.originalTheta = msg.data
	def velocity_callback(self, msg):
		self.velocity = msg.data
		sC.weightedFilter()

	def weightedFilter(self):
	 	alpha = 1
		yaw_degrees = self.total_IMU_yaw * (180/3.141592653)
	 	abs_del = abs(self.originalTheta-yaw_degrees)
		velocity_centre = self.velocity
		gainValue =5
		if velocity_centre >= 0.2:
		 	gainValue = (0.1/self.velocity)
	 	alphaQ = abs_del/(abs_del+gainValue)
	 	if abs_del > 0.1 and abs_del < 15:
	 		filterValue = alphaQ
	 	elif abs_del >= 15:
	 		filterValue = alpha
	 		print 'slip is being corrected...'
	 	else:
			filterValue = 0
		self.filteredTheta = (1-filterValue)*self.originalTheta + filterValue*self.total_IMU_yaw
		print 'encoder Theta ===========================:  ', self.originalTheta
		print 'yaw values ==============================:  ', yaw_degrees
	 	print 'filtered Theta ===========================: ', self.filteredTheta
	 	print 'outta weightedF'

if __name__ == '__main__':
	try:
		sC = fuser()
		sC.spin()
	except rospy.ROSInterruptException:pass
