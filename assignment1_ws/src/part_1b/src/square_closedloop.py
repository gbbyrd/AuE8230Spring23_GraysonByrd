#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import time

PI = 3.141592

class Square_Closedloop():
    def __init__(self) -> None:
        self.coordinates = [[5, 5],
                           [8, 5],
                           [8, 8],
                           [5, 8]
                           ]
        
        rospy.init_node('square_closedloop', anonymous=True)
        self.vel_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.rate = rospy.Rate(50)
        self.pose = Pose()
        self.pose_count = 0
        self.vel_msg = Twist()
        
        # Initialize velocity
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        
        # Set standard linear and angular velocites
        self.vel_linear = 1
        self.vel_angular = .25
        
        self.theta = 0
    
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)
        self.pose_count += 1
        if self.pose_count % 50 == 0:
            print(f'X: {self.pose.x}, Y: {self.pose.y}, Theta: {self.pose.theta}')
        
    def zero_velocity(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0    
        
    def go_straight(self, target, axis):
        
        self.vel_msg.linear.x = self.vel_linear
        
        if axis == 'x':
            while abs(self.pose.x - target) > 0.1:
                self.vel_publisher.publish(self.vel_msg)
        else:
            while abs(self.pose.y - target) > 0.1:
                self.vel_publisher.publish(self.vel_msg)
                
        self.zero_velocity()
                
    def orient(self, target_theta):
        
        self.vel_msg.angular.z = self.vel_angular
        
        while abs(self.pose.theta - target_theta) > 0.005:
            self.vel_publisher.publish(self.vel_msg)
            
        self.zero_velocity()
        
    def move_to_coord(self, target):
        
        x_target = target[0]
        y_target = target[1]
        
        # Move to x position
        if x_target - self.pose.x > 1:
            target_theta = 0
            self.orient(target_theta)
            self.go_straight(x_target, 'x')
        elif x_target - self.pose.x < -1:
            target_theta= - PI
            self.orient(target_theta)
            self.go_straight(x_target, 'x')
            
        # Move to y position
        if y_target - self.pose.y > 1:
            target_theta = PI / 2
            self.orient(target_theta)
            self.go_straight(y_target, 'y')
        elif y_target - self.pose.y < -1:
            target_theta = -(PI / 2)
            self.orient(target_theta)
            self.go_straight(y_target, 'y')
        
    def move(self):
        
        coord_index = 0
        time.sleep(3)
        while not rospy.is_shutdown():
            if coord_index > 3:
                coord_index = 0
            print(self.coordinates[coord_index])
            self.move_to_coord(self.coordinates[coord_index])
            coord_index += 1
            
if __name__ == '__main__':
    move = Square_Closedloop()
    move.move()
            
                
    
        