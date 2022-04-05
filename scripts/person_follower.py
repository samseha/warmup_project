#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

#How close we will get to person
distance = 0.4

class PersonFollower(object):
    """Follows a person while keeping a distance"""
    def __init__(self):
        #initialize ROS node
        rospy.init_node('person_follower')
        #setup publisher
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #scan topic and set self.process_scan as the call back fxn
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.vel = Twist()
    
    def best_angle(self, angle):
        if angle > 180:
            return angle - 180
        else:
            return angle

    def process_scan(self, data):
        #find closest object
        min_angle = 0
        min_value = math.inf
        rospy.loginfo("data.ranges", data.ranges)
        rospy.loginfo("length", len(data.ranges))
        for i in range(len(data.ranges)):
            if data.ranges[i] < min_value:
                min_value = data
                min_angle = i
        if min_value == math.inf:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
        else:
            if min_value >= distance:
                self.vel.linear.x = 0.1
                self.vel.angular.z = math.radians(self.best_angle(min_angle)))
            else:
                self.vel.linear.x = 0
                self.vel.angular.z = math.radians(self.best_angle(min_angle)))
        
        self.twist_pub.publish(self.vel)
    

    def run(self):
        # Keep the program alive.
        rospy.spin()
​
if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()
