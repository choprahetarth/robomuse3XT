#!/usr/bin/env python
import rospy
import time
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3

class box():
	def __init__(self):
		rospy.init_node("calibrate")
		nodename = rospy.get_name()
		rospy.loginfo("node %s started" % nodename)
		# subscribers
		rospy.Subscriber('robomuse_diff/odom', Odometry, self.odometry_callback)
		rospy.Subscriber('hokuyo_pose', Pose2D, self.laser_callback)
		rospy.Subscriber('imu', Float32, self.imu_callback)
		# publishers
		self.cmd = rospy.Publisher('calibrate/cmd_vel', Twist, queue_size = 10)
		self.factorVector = rospy.Publisher('calibrationFactors', Vector3, queue_size = 10)
		# variables
		self.rate = rospy.get_param("~rate", 10)
		#values for the box path function
		self.x = 0
		self.y = 0
		self.xlaser = 0
		self.thetalaser = 0
		self.ylaser = 0
		self.yaw = 0
		self.calibrateVel = Twist()
		self.start = time.time()
		self.count = 0
		self.loopFlag = 0
		self.calibration_protocol = 0
		self.xFactor = 0
		self.yFactor = 0
		self.thetaFactor = 0
		self.publishOnceFlag = 1
		self.factors = Vector3()
	def control_function(self):
		while not rospy.is_shutdown():
			cal.self_calibration()
			

	# callback functions
	def odometry_callback(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
	def laser_callback(self, msg):
		self.xlaser = msg.pose.position.x
		self.ylaser = msg.pose.position.y
		self.thetalaser = msg.pose.position.z
	def imu_callback(self,msg):
		self.yaw = msg.data

	# main calibration function
	def self_calibration(self):
		initialX = self.x
		initialY = self.y
		initialYaw = self.yaw
		while (self.calibration_protocol == 0):
			if rospy.is_shutdown():
				break
			while (abs(time.time() - self.start) < 2):
				#print(time.time() - self.start)
				if (self.count == 0):
					self.calibrateVel.linear.x = 0.3
					rospy.loginfo_throttle(120,"calibration started 0% ")
				elif (self.count == 1 or self.count == 2):
					self.calibrateVel.linear.x = -0.3
					rospy.loginfo_throttle(120,"calibration started 32% ")
				elif (self.count == 3):
					self.calibrateVel.linear.x = 0.3
					rospy.loginfo_throttle(120,"calibration started 50%")
				elif (self.count == 4):
					self.calibrateVel.linear.x = 0.0
					self.calibrateVel.angular.z = 0.4
					rospy.loginfo_throttle(120,"calibration started 65%")
				elif (self.count == 5):
					self.calibrateVel.linear.x = 0.0
					self.calibrateVel.angular.z = -0.4
					rospy.loginfo_throttle(120,"calibration started 80%")
				elif (self.count == 6):
					self.calibration_protocol = 1	
					rospy.loginfo_throttle(120,"calibration complete 100% ")
					break
				r = rospy.Rate(self.rate)
				self.cmd.publish(self.calibrateVel)
				r.sleep()		
			self.start = time.time ()
			self.count = self.count  +  1
		print("X ENCODER ==========> ", initialX)
		print("Y ENCODER ==========> ", initialY)
		print("THETA ==============>" , initialYaw)
		print("LASER X ============> ", self.xlaser)
		print("LASER Y ============> ", self.ylaser)
		print("LASER Z ============> ", self.thetalaser)
		self.xFactor = initialX/(self.xlaser+10)
		self.yFactor = initialY/(self.ylaser+10)
		self.thetaFactor = initialYaw/(self.thetalaser+10)
		self.factors = Vector3(self.xFactor , self.yFactor, self.thetaFactor)
		if (self.publishOnceFlag == 1):
			r = rospy.Rate(self.rate)
			self.factorVector.publish(self.factors)
			r.sleep()
			self.publishOnceFlag = 0
if __name__ == '__main__':
	try:
		cal = box()
		cal.control_function()
	except rospy.ROSInterruptException:pass
