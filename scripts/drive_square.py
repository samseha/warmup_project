#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class DriveSquare(object):
    """Uses a sleep time to make the robot drive in a square"""
    def __init__(self):
        #initilaize ROS node
        rospy.init_node("drive_square")
        #setup publisher to cmd_vel ROS topic
        self.square_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.vel = Twist()
        rospy.sleep(1)

    def square(self):
        while not rospy.is_shutdown():
            #set initial velocity to going forward with speed 0.1
            self.vel.linear.x = 0.1
            self.vel.linear.y = 0
            self.vel.linear.z = 0
            self.vel.angular.x = 0
            self.vel.angular.y = 0
            self.vel.angular.z = 0
            #publish message
            self.square_pub.publish(self.vel)
            rospy.loginfo("straight = %f", self.vel.linear.x)
            #sleep 10 second before turning
            rospy.sleep(10)

            #turn robot by 90 degrees in radians
            self.vel.linear.x = 0.0
            self.vel.angular.z = 1.5708
            #publish the message
            self.square_pub.publish(self.vel)
            rospy.loginfo("turning = %f", self.vel.angular.z)
            rospy.sleep(1)

if __name__ == '__main__':
    node = DriveSquare()
    node.square()

