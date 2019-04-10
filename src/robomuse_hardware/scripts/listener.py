#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def encoderCallback(msg):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", msg.data)

def imuCallback(msg):
    rospy.loginfo(rospy.get_caller_id() + "the imu value", msg.data)

    
def calculator():
    rospy.init_node('angleFuser', anonymous=True)   #listener ka node 
    rospy.Subscriber("normalTheta", Float32, encoderCallback)	# encoder da topic
    rospy.Subscriber("imu_in", Float32 , imuCallback)  # IMU da topic 
    rospy.spin()

if __name__ == '__main__':
    calculator()

