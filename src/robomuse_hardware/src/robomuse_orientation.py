#!/usr/bin/env python
# license removed for brevity
import rospy
import math
import time
from geometry_msgs.msg import Twist

time = 0
linear_vel = 0.25                                               #m/s
#linear_vel = 0.02   reduced velocity so that the motors dont get damaged
linear_to_ticks = 6161.47                                       #conversion factor
ticks_acc = 1000
ticks_deacc = 2000

def position(pos):
    global linear_vel
    if(pos < 0.5):
        linear_vel = 0.1                                        #m/s
    print "linear_vel is :",linear_vel
    total_ticks = pos*linear_to_ticks
    ticks_vel = linear_vel*linear_to_ticks                      #ticks/s
    time_acc = ticks_vel/ticks_acc
    distance_acc = (ticks_acc*time_acc*time_acc)/2              # Ticks covered to reach the velocity
    time_deacc = ticks_acc/ticks_deacc
    distance_deacc = (ticks_deacc*time_deacc*time_deacc)/2      # Ticks covered to reach zero velocity
    ticks_required = total_ticks - distance_acc - distance_deacc
    time_ticks_required = ticks_required/ticks_vel
    total_time = time_ticks_required + time_acc + time_deacc
    print "I am inside the loop for:",total_time
    return total_time

def robomuse_position():
    global time
    publisher_val = rospy.Publisher('robomuse_diff/cmd_vel', Twist, queue_size=10)
    rospy.init_node('robomuse_orientation', anonymous=True)
    rate = rospy.Rate(10)                               	# Rate --->10hz
    pos_cmdvel = Twist()
    print "Type the linear distance for the robot in metres:"
    linear_distance=input()
    while not rospy.is_shutdown():
        pos_cmdvel.linear.x=0
        pos_cmdvel.linear.y=0
        pos_cmdvel.linear.z=0
        pos_cmdvel.angular.x=0
        pos_cmdvel.angular.y=0
        pos_cmdvel.angular.z=0
        while(time < position(linear_distance)):
            #pos_cmdvel.linear.x = 0.25
	    pos_cmdvel.linear.x = linear_vel	
            publisher_val.publish(pos_cmdvel)
            rate.sleep()
            time = time + 0.1
            print time
        print "I have exit the loop in:",time,"sec"
        publisher_val.publish(pos_cmdvel)
        rate.sleep()

if __name__ == '__main__':
    try:
        robomuse_position()
    except rospy.ROSInterruptException:
        pass
