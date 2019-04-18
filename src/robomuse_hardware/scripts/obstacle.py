#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist

class obstacle():
	def __init__(self):
		rospy.init_node("obstacle_detection")
		nodename = rospy.get_name() # get the name of the node
		rospy.loginfo("node %s has started" % nodename)	# log that the node has successfully started
		#subscribers 
		rospy.Subscriber('obstacle_pin', Int16, self.obstacleCallback)
		#publisher
		self.commando = rospy.Publisher('obstacle/cmd_vel', Twist, queue_size = 20)
		#variables 
		self.status = 0.0
		self.rate = rospy.get_param("~rate", 20)
		self.command_vel = Twist()

	def interrupt(self):
		if(self.status == 1):
			self.command_vel.linear.x = 0.0
			self.command_vel.linear.y = 0.0
			self.command_vel.linear.z = 0.0
			self.command_vel.angular.x = 0.0
			self.command_vel.angular.y = 0.0
			self.command_vel.angular.z = 0.0
			self.commando.publish(self.command_vel)

	def obstacleCallback(self, msg):
		self.status = msg.data

	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			r.sleep()
			c.interrupt()


if __name__ == '__main__':
	try:
		c = obstacle()	
		c.spin()	
	except rospy.ROSInterruptException:pass
