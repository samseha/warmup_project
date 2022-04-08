#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

#How close we will get to wall
distance = 1
k = 0.2

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
                if dist < 1.5:
                    self.at_wall = True
                    break
        speed = k * data.ranges[0]
        if speed > 0.3:
            return 0.3
        return speed

    def get_angle(self, data):
        # if not self.at_wall:
        #     #go straight until wall
        #     return 0
        # #find which way to turn using trigonometry
        # min_dist = math.inf
        # min_angle = 0
        # turn_angle = 0
        # for i in range(len(data.ranges)):
        #     if min_dist > data.ranges[i] and data.ranges[i] != 0:
        #         min_dist = data.ranges[i]
        #         min_angle = i
        # rospy.loginfo("minangle: %f", min_angle)
        # if min_dist == math.inf:
        #     return 0
        # rospy.loginfo("mindist: %f, %f", min_dist, data.ranges[0])
        # if min_angle < 180:
        #     #wall on leftside
        #     if abs(data.ranges[0] - min_dist) < 0.35:
        #         return -90
        #     turn_angle = -(90 - (min_angle + 1))
        # else:
        #     #wall on right side
        #     if abs(data.ranges[0] - min_dist) < 0.35:
        #         return 90
        #     turn_angle = 359 - min_angle
        
        # return turn_angle
        if not self.at_wall:
            #go straight until wall
            return 0
        #find which way to turn using trigonometry
        front_dist = data.ranges[0]
        dist_45 = data.ranges[44]
        dist_90 = data.ranges[89]
        dist_135 = data.ranges[134]
        dist_180 = data.ranges[179]
        #will always go clockwise
        angle_diff = dist_135 - dist_45
        wall_ang = distance - dist_90

        if angle_diff == math.inf:
            return -dist_45
        elif angle_diff == -1 * math.inf:
            return dist_135
        else:
            return -(angle_diff + wall_ang)

    def process_scan(self, data):
        #process the scan and publish speed
        self.at_wall = False
        self.vel.linear.x = self.get_x(data)
        self.vel.angular.z = self.get_angle(data)
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
