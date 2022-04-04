#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

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
    
    #def process_scan(self, data):
