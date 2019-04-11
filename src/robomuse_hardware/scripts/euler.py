#!/usr/bin/env python 
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
import tf.transformations as trans


def convertor():
    rospy.init_node('qToEuler', anonymous = True) # listener ka node 
    rospy.Subscriber('imu_in', Imu , imuCallback) # imu ka subscriber with the imu message type 
    rospy.spin()

def yprPublisher():
    pub = rospy.Publisher('yawPitchRoll', Vector3 ,  queue_size = 20)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
	rospy.loginfo(ypr)
        pub.publish(ypr)
        rate.sleep()


def imuCallback(msg):
    q_in = [msg.orientation.x,
	    msg.orientation.y,
            msg.orientation.z,
	    msg.orientation.w]
    euler = trans.euler_from_quaternion(q_in) 
    ypr = Vector3(euler[2],
           euler[0],
	   euler[1])
    rospy.loginfo(ypr)

if __name__ == '__main__':
   convertor()
   yprPublisher()

