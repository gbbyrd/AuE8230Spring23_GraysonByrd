#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def move_in_circle():
    # Start the node
    rospy.init_node('move_circle', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    vel_msg = Twist()
    
    # Set the velocities
    vel_msg.linear.x = 1
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 1

    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)
        

if __name__ == '__main__':
    try:
        move_in_circle()
    except rospy.ROSInterruptException: pass