#!/usr/bin/env python
import rospy
from array import array
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf.transformations as trans

class converterClass():
	def __init__(self):
		rospy.init_node("converter")  # here we start the node
		nodename = rospy.get_name() # get the name of the node
		rospy.loginfo("node %s has started" % nodename)	# log that the node has successfully started
		# subscribers
		rospy.Subscriber('imu_in', Imu, self.imuCallback)  # this subscribes the data from the imu
		# publishers
		self.eulerAngles = rospy.Publisher('yprEuler', Vector3, queue_size = 10) # this
		# variables
		self.q_in = []
		self.euler = 0
		self.ypr = Vector3()
		self.rate = rospy.get_param("~rate", 20)

	def spin(self):
		r = rospy.Rate(self.rate)
		while not rospy.is_shutdown():
			self.spinOnce()
			r.sleep()

	def spinOnce(self):
		self.eulerAngles.publish(self.ypr)



	# callback functions
	def imuCallback(self, msg):
		self.q_in = [msg.orientation.x,
	    		msg.orientation.y,
            	msg.orientation.z,
	    		msg.orientation.w]
    		self.euler = trans.euler_from_quaternion(self.q_in)
    		self.ypr = Vector3(self.euler[2],
           		self.euler[0],
	   			self.euler[1])

if __name__ == '__main__':
	try:
		C = converterClass()
		C.spin()
	except rospy.ROSInterruptException:pass
