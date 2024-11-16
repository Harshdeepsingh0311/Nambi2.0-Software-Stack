#!/usr/bin/env python

import rospy
from serial import Serial
from std_msgs.msg import String

def main():
    rospy.init_node('gps_serial_reader')
    serial_port = Serial('/dev/ttyACM0', 4800)  # Replace with the correct serial port and baud rate

    pub = rospy.Publisher('gps_data', String, queue_size=10)

    rate = rospy.Rate(3)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        gps_data = serial_port.readline()
        pub.publish(gps_data.strip().decode('utf-8'))  # Remove any leading/trailing whitespace
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

