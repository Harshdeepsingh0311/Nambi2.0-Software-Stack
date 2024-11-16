#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray, Float64MultiArray
import numpy as np


class sensor_data_pub:
    def __init__(self):
        self.counter_Acc = 0
        self.counter_GPS = 0
        self.counter_IMU = 0
        self.max_counter = 50
        self.Acc_data = []
        self.IMU_data = []
        self.GPS_data = []

    def fetchAccData(self, single_obs):
        if self.counter_Acc == self.max_counter:
            print("Acc")

            print(np.cov(np.transpose(np.array(self.Acc_data))))
            self.counter_Acc = 0
            # self.Acc_data = []
        else:
            self.counter_Acc += 1
            self.Acc_data.append(single_obs.data)

    def fetchGPSData(self, single_obs):
        if self.counter_GPS == self.max_counter:
            print("GPS")
            print(np.cov(np.transpose(np.array(self.GPS_data))))
            self.counter_GPS = 0
            # self.GPS_data = []
        else:
            self.counter_GPS += 1
            self.GPS_data.append(single_obs.data)

    def fetchIMUData(self, single_obs):
        if self.counter_IMU == self.max_counter:
            print("IMU")
            print(np.cov(np.transpose(np.array(self.IMU_data))))
            self.counter_IMU = 0
            # self.IMU_data = []
        else:
            self.counter_IMU += 1
            self.IMU_data.append(single_obs.data[:-1])

    # def getCovariance(observations):
    # return np.cov(observations)

    def listener(self):
        rospy.init_node("Sensor_data")
        rospy.Subscriber("IMU", Float32MultiArray, self.fetchIMUData)
        rospy.Subscriber("Acc", Float64MultiArray, self.fetchAccData)
        rospy.Subscriber("LatLon", Float64MultiArray, self.fetchGPSData)
        rospy.spin()


if __name__ == "__main__":
    data_pub = sensor_data_pub()
    data_pub.listener()
