#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

#How close we will get to wall
distance = 1
k = 1.1

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
        if not self.at_wall:
            #not at wall go to wall
            speed = k * (data.ranges[0] - distance)
            return speed
        else:
            #at wall proceed with wall following speed
            return 0.3

    def get_angle(self, data):
        if not self.at_wall:
            #go straight until wall
            return 0
        #find which way to turn using trigonometry
        min_dist = data.range[0]
        min_angle = 0
        turn_angle = 0
        for i in range(len(data.range)):
            if min_dist > data.range[i]:
                min_dist = data.range[i]
                min_angle = i
        if min_angle < 180:
            #wall on leftside
            turn_angle = -(90 - (min_angle + 1))
        else:
            #wall on right side
            turn_angle = 90 - (359 - min_angle)
        
        return turn_angle

    def process_scan(self, data):
        #process the scan and publish speed
        self.vel.linear.x = self.get_x(data)