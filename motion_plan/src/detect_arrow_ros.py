#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images

import sys
import argparse
import numpy as np
import cv2
import os, glob, time

from detect_arrow_webcam import *

ROS_TOPIC = "/realsense/color/image_raw"  #'/mrt/camera/color/image_raw'


class ImageSubscriber:

    """Subscribes to ROS Topic and calls image_callback"""

    def __init__(self, image_topic):
        """

        :image_topic: string

        """
        rospy.init_node("image_sub", anonymous=True)
        self.br = CvBridge()
        self.sub = rospy.Subscriber(image_topic, Image, self.image_callback)
        self.font = cv2.FONT_HERSHEY_SIMPLEX
        # Blue color in BGR
        self.color = (255, 255, 0)
        self.org = (50, 50)
        self.fontScale = 1
        self.thickness = 2
        self.vid_file = cv2.VideoWriter(
            "arrow.mp4", cv2.VideoWriter_fourcc(*"MP4V"), 10, (640, 480)
        )
        rospy.spin()
        print("all done!")
        self.vid_file.release()
        cv2.destroyAllWindows()

    def image_callback(self, data):
        """Converts ROS Image, passes to arrow_detect and displays detected

        :data: Image
        :returns: None

        """
        cv_img = self.br.imgmsg_to_cv2(data)
        found, theta, orient, direction, output = arrow_detect(cv_img)
        print("shape: ", output.shape)

        if direction == 1:
            direction = "Right"
        elif direction is None:
            direction = "not found"
        else:
            direction = "Left"

        output = cv2.putText(
            output,
            direction,
            self.org,
            self.font,
            self.fontScale,
            self.color,
            self.thickness,
            cv2.LINE_AA,
        )

        self.vid_file.write(output)
        cv2.imshow("Arrow", output)
        cv2.waitKey(20)
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        #     break


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t", "--topic", help="ROS Topic to subscribe to", default=ROS_TOPIC
    )
    args = parser.parse_args()

    subscriber = ImageSubscriber(args.topic)
    # cv2.destroyAllWindows()
