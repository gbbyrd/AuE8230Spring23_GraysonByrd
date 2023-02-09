#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

PI = 3.141592

class Square_Openloop():
    def __init__(self) -> None:
        #Initialize the node and publisher
        #Anonymous is used if you expect there to be a lot of this node
        #and the name is not important. It automatically adds a random number
        #to the end of the node name
        rospy.init_node('square_openloop', anonymous=True)
        #Queue size limits the amounts of queued messages if the topic cannot
        #recieve/process them fast
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist)
        self.vel_msg = Twist()
        
        self.linear_vel = 0.5
        self.angular_vel = -0.5
        
        # Set velocity
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.linear.z = 0
        self.vel_msg.angular.x = 0
        self.vel_msg.angular.y = 0
        self.vel_msg.angular.z = 0
        
    def zero_velocity(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.angular.z = 0
        self.velocity_publisher.publish(self.vel_msg)
        
    def go_straight(self):
        duration = 2 / 0.2
        self.vel_msg.linear.x = self.linear_vel
        t1 = rospy.Time.now().to_sec()
        t2 = t1
        while (t2 - t1 < duration):
            self.velocity_publisher.publish(self.vel_msg)
            t2 = rospy.Time.now().to_sec()
        self.zero_velocity()
            
    def rotate_cw(self):
        angular_vel = self.angular_vel
        t1 = rospy.Time.now().to_sec()
        t2 = t1
        # Calculate time for 1/4 rotation, or PI/2
        duration = (PI/2)/abs(angular_vel)
        self.vel_msg.angular.z = angular_vel
        while (t2 - t1 < duration):
            self.velocity_publisher.publish(self.vel_msg)
            t2 = rospy.Time.now().to_sec()
        self.zero_velocity()
        
    def move_in_square(self):
        
        rospy.on_shutdown(self.shutdown)
        
        while not rospy.is_shutdown():
            self.go_straight()
            self.rotate_cw()
            
    def shutdown(self):
        rospy.loginfo('Stop TurtleBot: Node shutdown...')
        
        self.velocity_publisher.publish(Twist())
        
        rospy.sleep(1)
                
if __name__ == '__main__':
    move = Square_Openloop()
    move.move_in_square()
            
        
        
    
    
    