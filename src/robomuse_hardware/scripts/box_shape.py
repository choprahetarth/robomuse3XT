#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry

class box():
	def __init__(self):
		rospy.init_node("box_shape")
		nodename = rospy.get_name()
		rospy.loginfo("node %s started." % nodename)
		# subscribers
		rospy.Subscriber('robomuse_diff/odom', Odometry, self.distance_callback)
		rospy.Subscriber('normalTheta', Float32, self.theta_callback)
		# publishers
		self.cmd = rospy.Publisher('autonomous/cmd_vel', Twist, queue_size = 20)
		# variables
		self.rate = rospy.get_param("~rate", 20)
		# values for the box path function
		self.originalTheta = 0.0
		self.angles = [90,180,270,360]
		self.x = 0.0
		self.y = 0.0
		self.distance = 0.0
		self.oldDistance = 0.0
		self.newDistance = 0.0
		self.distanceIncrement = 0.0
		self.command_vel = Twist()
		self.currentDistance = 0.0
		self.oldTheta = 0.0
		self.newTheta = 0.0
		self.thetaIncrement = 0.0

	# callback functions
	def distance_callback(self, msg):
		self.x = msg.pose.pose.position.x
		self.y = msg.pose.pose.position.y
		self.distance = math.sqrt((self.x*self.x)+(self.y*self.y))
		self.newDistance = self.distance
		self.distanceIncrement = self.newDistance - self.oldDistance
		self.oldDistance = self.newDistance
	def theta_callback(self, msg):
		self.originalTheta = msg.data
		self.newTheta = self.originalTheta
		self.thetaIncrement = self.newTheta - self.oldTheta
		self.oldTheta = self.newTheta
	def boxPath(self):
		displacement = input('Total displacement: ')
		nloops =0
		resetValue=0.0
		self.currentDistance = resetValue
		self.currentTheta = resetValue
		while (nloops < 4):
			while (abs(self.currentDistance) < displacement):
				self.command_vel.linear.x = 0.3
				self.command_vel.linear.y = 0.0
				self.command_vel.linear.z = 0.0
				self.command_vel.angular.x = 0.0
				self.command_vel.angular.y = 0.0
				self.command_vel.angular.z = 0.0
				self.currentDistance = self.currentDistance + self.distanceIncrement
				if(abs(self.currentDistance - displacement)<0.05):
					self.command_vel.linear.x = 0.15
				self.cmd.publish(self.command_vel)
				rospy.Rate(self.rate).sleep()
				#r = rospy.Rate(self.rate)  #### publishing
				#while not rospy.is_shutdown():
					#r.sleep()
				print(self.currentDistance)
			sC.resetXY()
			print("exited")
			nloops = nloops + 1
			exitFlag = 1
			#then travel to the loop of the angle

			while (self.currentTheta < self.angles[nloops-1]):
				self.currentTheta = self.currentTheta + self.thetaIncrement
				print(self.currentTheta)
				self.command_vel.linear.x = 0.0
				self.command_vel.angular.z = 0.2
				if (abs(self.currentTheta - self.angles[nloops-1]) <10):
					self.command_vel.angular.z = 0.1
				self.cmd.publish(self.command_vel)
				rospy.Rate(self.rate).sleep()
		sC.resetTheta()

	def resetXY(self):
		print("i am here")
		self.currentDistance = 0.0
	def resetTheta(self):
		self.currentTheta = 0.0


if __name__ == '__main__':
	try:
		i = 0
		sC = box()
		#sC.spin()
		turns = input("total number of iterations: ")
		while(i < turns): 
			sC.boxPath()
			i= i+1
	except rospy.ROSInterruptException:pass
