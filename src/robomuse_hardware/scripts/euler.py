#!/usr/bin/env python 
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu
import tf.transformations as trans

def convertor():
    rospy.init_node('qToEuler', anonymous = True) # listener ka node 
    rospy.Subscriber('imu_in', Imu , imuCallback) # imu ka subscriber with the imu message type 
    rospy.spin()

def imuCallback(msg):
    q_in = [msg.orientation.x,
	    msg.orientation.y,
            msg.orientation.z,
	    msg.orientation.w]
    euler = trans.euler_from_quaternion(q_in) 
    ypr = [euler[0],
           euler[1],
	   euler[2]]
    rospy.loginfo(ypr)
if __name__ == '__main__':
   convertor()



