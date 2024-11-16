#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import numpy as np

radius = 0.4375


class motor_cmd:
    def __init__(self):
        rospy.init_node("motor_cmd", anonymous=True)
        self.sub = rospy.Subscriber("/cmd_vel_filtered", Twist, self.callback)
        self.pub = rospy.Publisher("/motor_cmd", Float32MultiArray, queue_size=10)
        rospy.spin()

    def callback(self, data):
        global radius
        array = np.zeros(4)  # first two for right side, other two for left side
        array[0] = data.linear.x + data.angular.z * radius
        array[1] = data.linear.x + data.angular.z * radius
        array[2] = data.linear.x - data.angular.z * radius
        array[3] = data.linear.x - data.angular.z * radius
        self.pub.publish(Float32MultiArray(data=array))


if __name__ == "__main__":
    try:
        my_motor_cmd = motor_cmd()
    except rospy.ROSInterruptException:
        print("Closing motor_cmd")
