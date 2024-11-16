#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import NavSatFix, Imu, NavSatStatus
from geometry_msgs.msg import Quaternion, Vector3
from tf.transformations import quaternion_from_euler

ROOT_LINK = "root_link"


class sensor_data_pub:
    def __init__(self):
        self.acc = Vector3()
        self.q = Quaternion(0, 0, 0, 1)
        self.gps = NavSatFix()
        self.omega = Vector3()

        rospy.init_node("Sensor_data", anonymous=True)
        rospy.Subscriber("IMU", Float64MultiArray, self.callback_IMU)
        rospy.Subscriber("Acc", Float64MultiArray, self.callback_Acc)
        rospy.Subscriber("LatLon", Float64MultiArray, self.callback_LatLon)
        rospy.Subscriber("Omega", Float64MultiArray, self.callback_Omega)
        self.imu_pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
        self.gps_pub = rospy.Publisher("/gps/fix", NavSatFix, queue_size=10)
        rospy.spin()

    def callback_IMU(self, data):
        euler_angle = data.data[:-1]
        roll = euler_angle[1]
        pitch = -euler_angle[0]
        euler_angle = roll, pitch, euler_angle[2]

        rospy.loginfo("I heard" + str(data.data))
        self.q = Quaternion(*quaternion_from_euler(*euler_angle))
        # self.publish_imu()
        # rospy.loginfo("The quaternion representation is %s %s %s %s." % (q[0], q[1], q[2], q[3]))[0], euler_angle[1], euler_angle[2]

    def callback_Acc(self, data):
        self.acc = Vector3(*data.data)
        # self.publish_imu()
        # rospy.loginfo("I heard"+str(data.data))
        # rospy.loginfo("The acceleration representation is %s %s %s." % (acc[0], acc[1], acc[2]))

    def callback_Omega(self, data):
        self.omega = Vector3(*data.data)
        self.publish_imu()

    def callback_LatLon(self, data):
        latlon = data.data
        # rospy.loginfo("I heard"+str(data.data))
        # rospy.loginfo("The GPS representation is %s %s." % (latlon[0], latlon[1]))
        self.lat, self.lon = latlon
        self.publish_gps()

    def publish_gps(self):
        self.gps = NavSatFix()
        self.gps.header.frame_id = ROOT_LINK
        self.gps.header.stamp = rospy.Time.now()
        self.gps.latitude = self.lat
        self.gps.longitude = self.lon
        self.gps.position_covariance = [
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
        ]
        self.gps.status = NavSatStatus(
            status=NavSatStatus.STATUS_SBAS_FIX, service=NavSatStatus.SERVICE_GPS
        )
        self.gps.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(self.gps)

    def publish_imu(self):
        self.imu = Imu()
        self.imu.header.frame_id = "imu_link"
        self.imu.header.stamp = rospy.Time.now() + rospy.Duration(nsecs=2e8)

        self.imu.orientation = self.q
        self.imu.orientation_covariance = [
            2.55441508e-08,
            8.39203826e-09,
            -5.98954506e-08,
            8.39203826e-09,
            2.71482299e-08,
            2.95416336e-08,
            -5.98954506e-08,
            2.95416336e-08,
            9.50042071e-05,
        ]  # [1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3]
        self.imu.linear_acceleration = self.acc
        self.imu.angular_velocity = self.omega
        self.imu.linear_acceleration_covariance = [
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
        ]
        self.imu.angular_velocity_covariance = [
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
            1e-3,
        ]
        self.imu_pub.publish(self.imu)


if __name__ == "__main__":
    data_pub = sensor_data_pub()
