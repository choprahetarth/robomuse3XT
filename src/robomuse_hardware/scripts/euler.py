#!/usr/bin/env python 
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf.transformations as trans

class converterClass():
	def __init__(self):
		rospy.init_node("converter")
		nodename = rospy.get_name()
		rospy.loginfo("node %s has started" % nodename)
		# subscribers
		rospy.Subscriber('imu_in', Imu, self.imuCallback)  # change the name of the topic and the varible 

		# publishers
		self.eulerAngles = rospy.Publisher('yprEuler', Vector3, queue_size = 10) # change the name of the topic and the data type name 
	
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
    		self.euler = trans.euler_from_quaternion(q_in) 
    		self.ypr = Vector3(euler[2],
           			euler[0],
	   			euler[1])
		rospy.loginfo(ypr)
	

if __name__ == '__main__':
	try:
		C = converterClass()
		C.spin()
	except rospy.ROSInterruptException:pass
