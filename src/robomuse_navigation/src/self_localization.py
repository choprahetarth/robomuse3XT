#!/usr/bin/env python
import rospy
import os
import time
from geometry_msgs.msg import Twist
from math import radians
print("ho")
def localize ():
    rospy.init_node('self_localize', anonymous=True)
    velocity_publisher = rospy.Publisher('robomuse_diff/cmd_vel', Twist, queue_size=10)
    turn_cmd = Twist()
    os.system('rosservice call /global_localization "{}" ')
    print("Let's rotate")
    angular_speed = radians(60)
    relative_angle = radians(1080)
    turn_cmd.angular.z = -abs(angular_speed)

    t0 = rospy.Time.now().to_sec()
    current_angle = 0

    while(current_angle < relative_angle):
        velocity_publisher.publish(turn_cmd)
        t1 = rospy.Time.now().to_sec()
        current_angle = angular_speed*(t1-t0)


    #Forcing our robot to stop
    turn_cmd.angular.z = 0
    velocity_publisher.publish(turn_cmd)
    rospy.spin()

if __name__ == '__main__':
    try:
        #Testing our function
        localize()
    except rospy.ROSInterruptException: pass
