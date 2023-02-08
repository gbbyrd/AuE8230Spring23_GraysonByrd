#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Circle():
    
    def __init__(self) -> None:
        # Start the node
        rospy.init_node('move_circle', anonymous=False)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel_msg = Twist()
        
        # Set the velocities
        self.vel_msg.linear.x = .25
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = .25
        
    def move_in_circle(self):
        
        
        rospy.on_shutdown(self.shutdown)
        
        while not rospy.is_shutdown():
            self.velocity_publisher.publish(self.vel_msg)
        
        rospy.spin()

    def shutdown(self):
        rospy.loginfo("Stop Turtlebot")

        self.velocity_publisher.publish(Twist())

        rospy.sleep(1)

if __name__ == '__main__':
    circle = Circle()
    try:
        circle.move_in_circle()
    except rospy.ROSInterruptException: pass
