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
		rospy.Subscriber('imu', Float32, self.imu_callback)
		rospy.Subscriber('normalTheta', Float32, self.theta_callback)
		rospy.Subscriber('velo_centre', Float32, self.velocity_callback)
		rospy.Subscriber('lmotor_cmd', Float32, self.leftMotor_cmdCallback)
		rospy.Subscriber('rmotor_cmd', Float32, self.rightMotor_cmdCallback)
		# publishers
		self.filter = rospy.Publisher('filteredTheta', Float32, queue_size = 20)
		# variables
		self.rate = rospy.get_param("~rate", 20)
		# values for the filter
		self.velocity = 0.0
		self.originalTheta = 0.0
		self.total_IMU_yaw = 0.0
		self.filteredTheta = 0.0
		self.newIMUyaw = 0.0
		self.oldIMUyaw  =0.0
		self.deltaIMUyaw = 0.0
		self.imuYawSum = 0.0
		self.newOriginalTheta = 0.0
		self.oldOriginalTheta = 0.0
		self.deltaOriginalTheta = 0.0
		self.originalThetaSum =0.0
		self.leftMotorValue = 0.0
		self.rightMotorValue = 0.0
	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.spinOnce()
			r.sleep()
	def spinOnce(self):
		self.filter.publish(self.filteredTheta)
	# callback functions
	def imu_callback(self, msg):
		self.total_IMU_yaw = msg.data *(-1)
		self.newIMUyaw= self.total_IMU_yaw
		self.deltaIMUyaw = self.newIMUyaw - self.oldIMUyaw
		self.imuYawSum = self.imuYawSum + self.deltaIMUyaw
		self.oldIMUyaw = self.newIMUyaw

	def theta_callback(self, msg):
		self.originalTheta = msg.data
		self.newOriginalTheta = self.originalTheta
		self.deltaOriginalTheta = self.newOriginalTheta - self.oldOriginalTheta
		self.originalThetaSum = self.originalThetaSum + self.deltaOriginalTheta 
		self.oldOriginalTheta = self.newOriginalTheta

	def velocity_callback(self, msg):
		self.velocity = msg.data
		sC.weightedFilter()
	def leftMotor_cmdCallback(self,msg):
		self.leftMotorValue = msg.data
	def rightMotor_cmdCallback(self,msg):
		self.rightMotorValue = msg.data


	def weightedFilter(self):
		if (self.rightMotorValue == 0.0):
			self.imuYawSum = 0.0
			self.originalThetaSum = 0.0
	 	alpha = 1
		yaw_degrees = self.imuYawSum
	 	abs_del = abs(self.originalThetaSum-yaw_degrees)
		velocity_centre = self.velocity
		gainValue =5
		#if velocity_centre >= 0.3:
		 	#gainValue = (0.1/self.velocity)
	 	alphaQ = abs_del/(abs_del+gainValue)
	 	if abs_del >= 0 and abs_del < 15:
	 		filterValue = alphaQ
	 	elif abs_del >= 15:
	 		filterValue = alpha
	 	#else:
			#filterValue = 0
		self.filteredTheta = (1-filterValue)*self.imuYawSum + filterValue*self.originalTheta
		print 'encoder Theta ===========================:', self.originalThetaSum
		print 'yaw values ==============================:', yaw_degrees
	 	print 'filtered Theta ===========================:', self.filteredTheta
	 	print 'outta weightedF'

if __name__ == '__main__':
	try:
		sC = fuser()
		sC.spin()
	except rospy.ROSInterruptException:pass
