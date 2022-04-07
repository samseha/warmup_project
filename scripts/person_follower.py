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
        #Finds the best angle to turnto
        #the robot became very gittery so to prevent it blocks turning if below 10
        if angle < 10:
            return 0
        #if angle is bigger than 180 we subtract it so it would not do exessive turns
        if angle > 180:
            return angle - 360
        else:
            return angle

    def process_scan(self, data):
        #find closest object
        min_angle = 0
        min_value = math.inf
        for i in range(len(data.ranges)):
            #check for min value and also exclude 0 as the min value
            #since 0 can be from erroneous readings
            if data.ranges[i] < min_value and data.ranges[i] != 0:
                min_value = data.ranges[i]
                min_angle = i
        rospy.loginfo("distance = %f", data.ranges[min_angle])
        rospy.loginfo("angle = %d", min_angle)
        #if there is no object stay still (was in gazebo, but not sure for real bots (is it just 0?))
        if min_value == math.inf:
            self.vel.linear.x = 0
            self.vel.angular.z = 0
        else:
            #if bigger than min distance move towards it
            if min_value > distance:
                self.vel.linear.x = 0.1
                self.vel.angular.z = math.radians(self.best_angle(min_angle))
            #if smaller than min distance stop
            else:
                self.vel.linear.x = 0
                self.vel.angular.z = math.radians(self.best_angle(min_angle))
        #publish speed
        self.twist_pub.publish(self.vel)
    

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = PersonFollower()
    node.run()
