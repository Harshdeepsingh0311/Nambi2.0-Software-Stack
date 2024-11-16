#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from rover_msgs.msg import drive_msg

MAX_LINEAR = 0.3
MAX_ANGULAR = 0.3

class filter():
    """Filters velocity and publishes drive_msg"""
    def __init__(self):
        rospy.init_node('vel_filter', anonymous=True)
        self.sub = rospy.Subscriber('/smooth_cmd_vel', Twist, self.callback)
        self.pub = rospy.Publisher('/rover/drive_directives/autonomous',drive_msg,queue_size=10)
        rospy.spin()

    def callback(self, data):
        drive = drive_msg()
        drive.mode = "autonomous"
        data.angular.z = min(abs(data.angular.z), MAX_ANGULAR)*np.sign(data.angular.z)
        data.linear.x = min(abs(data.linear.x), MAX_LINEAR)*np.sign(data.linear.x)
        if abs(data.angular.z) > 0.15 or abs(data.linear.x) + 0.05 <abs(data.angular.z):
            drive.direction = "anticlockwise" if data.angular.z>0 else "clockwise"
            drive.speed = 50*abs(data.angular.z)/MAX_ANGULAR
        else :
            drive.direction = "forward" if data.linear.x>0 else "backward"
            drive.speed = 100*abs(data.linear.x)/MAX_LINEAR
        self.pub.publish(drive)

if __name__ == '__main__':
    try:
        my_filter = filter()
    except rospy.ROSInterruptException:
        print("Closing vel_filter")
