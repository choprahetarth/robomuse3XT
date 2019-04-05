#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Int32

def talker():
    pub_l = rospy.Publisher('lwheel', Int32, queue_size=10)
    pub_r = rospy.Publisher('rwheel', Int32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        left_encoder=20
        right_encoder=20
        #hello_str = "hello world %s" % rospy.get_time()
        #//rospy.loginfo(hello_str)
        pub_l.publish(left_encoder)
        pub_r.publish(right_encoder)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
