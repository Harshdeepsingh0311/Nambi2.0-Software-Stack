#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
import numpy as np
from tf.transformations import quaternion_from_euler
import tf

BASE_WIDTH = 0.4375 * 2  # base_width/2*2
ROOT_LINK = "root_link"

# class odom_cmd():
#   def __init__(self):
#       rospy.init_node('odom_pub', anonymous=True)
#       self.sub = rospy.Subscriber('/wheel_output', Twist, self.callback)
#       self.pub = rospy.Publisher('/odom',Float32MultiArray,queue_size=10)
#       rospy.spin()

#   def callback(self, msg):
#       array = np.array(msg.data)
#       array = np.min(array, 10)#ig sometimes odom values are wildly inaccurate, i saw a similar condition somewhere.. remove if useless


class EncoderOdom:
    def __init__(self):
        rospy.init_node("odom_pub", anonymous=True)
        self.sub = rospy.Subscriber("/cmd_vel_filtered", Twist, self.vel_update)
        self.BASE_WIDTH = BASE_WIDTH
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = rospy.Time.now()
        rospy.spin()

    @staticmethod
    def normalize_angle(angle):
        while angle > np.pi:
            angle -= 2.0 * np.pi
        while angle < -np.pi:
            angle += 2.0 * np.pi
        return angle

    def vel_update(self, msg):
        current_time = rospy.Time.now()
        d_time = (current_time - self.last_enc_time).to_sec()
        self.last_enc_time = current_time

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = msg.linear.x
            vel_y = msg.linear.y
            vel_theta = msg.angular.z

        # dist_left = left_ticks / self.TICKS_PER_METER
        # dist_right = right_ticks / self.TICKS_PER_METER
        # dist = (dist_right + dist_left) / 2.0

        # # TODO find better what to determine going straight, this means slight deviation is accounted
        # if left_ticks == right_ticks:
        #     d_theta = 0.0
        #     self.cur_x += dist * cos(self.cur_theta)
        #     self.cur_y += dist * sin(self.cur_theta)
        # else:
        #     d_theta = (dist_right - dist_left) / self.BASE_WIDTH
        #     r = dist / d_theta
        #     self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
        #     self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))

        delta_x = (
            vel_x * np.cos(self.cur_theta) - vel_y * np.sin(self.cur_theta)
        ) * d_time
        delta_y = (
            vel_x * np.sin(self.cur_theta) + vel_y * np.cos(self.cur_theta)
        ) * d_time
        d_theta = vel_theta * d_time

        self.cur_x += delta_x
        self.cur_y += delta_y
        self.cur_theta += d_theta
        self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)
        self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)
        return vel_x, vel_theta

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = quaternion_from_euler(0, 0, cur_theta)
        current_time = rospy.Time.now()

        br = tf.TransformBroadcaster()
        br.sendTransform(
            (cur_x, cur_y, 0),
            quaternion_from_euler(0, 0, -cur_theta),
            current_time,
            ROOT_LINK,
            "odom",
        )

        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01  # 99999
        odom.pose.covariance[21] = 0.01  # 99999
        odom.pose.covariance[28] = 0.01  # 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = ROOT_LINK
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)


if __name__ == "__main__":
    try:
        my_odom_pub = EncoderOdom()
    except rospy.ROSInterruptException:
        print("Closing odom_pub")
