#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32


def anglePublisher():
    pub = rospy.Publisher('normalTheta', Float32, queue_size=20)
    rospy.init_node('angle1', anonymous=True)
    rate = rospy.Rate(20) # 10hz
    while not rospy.is_shutdown():
        angleNormal = 10
        rospy.loginfo(angleNormal)
        pub.publish(angleNormal)
        rate.sleep()

if __name__ == '__main__':
    try:
        anglePublisher()
    except rospy.ROSInterruptException:
        pass

