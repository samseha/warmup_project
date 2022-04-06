#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

#How close we will get to wall
distance = 0.7
k = 0.4

class WallFollower(object):
    """Navigates to wall and drive alongside wall"""
    def __init__(self):
        #initialize ROS node
        rospy.init_node('person_follower')
        #setup publisher
        self.twist_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        #scan topic and set self.process_scan as the call back fxn
        rospy.Subscriber("/scan", LaserScan, self.process_scan)
        self.vel = Twist()
        self.at_wall = False
    
    def get_x(self, data):
        if not self.at_wall:
            #check if robot is not at wall
            for dist in data.ranges:
                #check if there is a wall nearby
                if dist < 1.2:
                    self.at_wall = True
                    break
        if not self.at_wall or data.ranges[0] == math.inf:
            #not at wall go to wall
            speed = 0.3
            return speed
        else:
            #at wall proceed with wall following speed
            speed = k * (data.ranges[0] - distance)
            if speed > 0.3:
                return 0.3
            return speed

    def get_angle(self, data):
        if not self.at_wall:
            #go straight until wall
            return 0
        #find which way to turn using trigonometry
        min_dist = data.ranges[0]
        min_angle = 0
        turn_angle = 0
        for i in range(len(data.ranges)):
            if min_dist > data.ranges[i]:
                min_dist = data.ranges[i]
                min_angle = i
        rospy.loginfo("minangle: %f", min_angle)
        if min_dist == math.inf:
            return 0
        if min_angle < 180:
            #wall on leftside
            turn_angle = -(90 - (min_angle + 1))
        else:
            #wall on right side
            turn_angle = 90 - (359 - min_angle)
        
        return turn_angle

    def process_scan(self, data):
        #process the scan and publish speed
        self.at_wall = False
        self.vel.linear.x = self.get_x(data)
        self.vel.angular.z = math.radians(self.get_angle(data))
        rospy.loginfo("x = %f", self.vel.linear.x)
        rospy.loginfo("data = %f", data.ranges[0])
        rospy.loginfo("angle = %f", math.degrees(self.vel.angular.z))
        self.twist_pub.publish(self.vel)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = WallFollower()
    node.run()
