#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

def encoderCallback(msg):
    rospy.loginfo("%s", msg.data)

def yawCallback(msg):
    rospy.loginfo(rospy.get_caller_id() + "the yaw value", msg.data)

    
def calculator():
    rospy.init_node('angleFuser', anonymous=True)   #listener ka node 
    rospy.Subscriber("normalTheta", Float32, encoderCallback)	# encoder da topic
    rospy.Subscriber("yawValue", Float32 , yawCallback)  # IMU da topic 
    rospy.spin()

if __name__ == '__main__':
    calculator()

