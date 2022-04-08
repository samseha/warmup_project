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
                if dist < 1.2 and dist != 0:
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

        """ABOVE is my initial approach to this problem. I tried to find the exact angle
        to turn to by using minimum distance. However, due to friction and the robot not
        being perfect sometimes there would be errors in turning corners. The distance 
        between the front wall and the left wall while making turns were not identical
        causing problems. So instead of finding exact measurements I decided to take a 
        more calibrating approach."""
        
        # return turn_angle
        if not self.at_wall:
            #go straight until wall
            return 0
        #find which way to turn using distance from left of robot at
        #45, 90, 135 degrees
        dist_45 = data.ranges[44]
        dist_90 = data.ranges[89]
        dist_135 = data.ranges[134]
        #will always go clockwise once finding a wall
        angle_diff = dist_135 - dist_45
        wall_ang = distance - dist_90
        #adding math.inf for gazebo and 0 for real turtlebot
        #if we can not find left_back side of robot we need to turn
        #right until we can find it.
        if angle_diff == math.inf or dist_135 == 0:
            return -dist_45
        #if we cannot front left side of robot we need to turn left
        #until we find it
        elif angle_diff == -1 * math.inf or dist_45 == 0:
            return dist_135
        #else we use the angle difference and distance towards wall to go along wall and
        #turn at corners.
        else:
            return -(angle_diff + wall_ang)

    def process_scan(self, data):
        #process the scan and publish speed
        #Also always check if near a wall before getting velocity and angle
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
