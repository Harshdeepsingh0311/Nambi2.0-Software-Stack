#!/usr/bin/env python
# Import the necessary libraries
import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import Header
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import os, re

FRAME='camera_link_optical'

class ImagePublisher:
    def __init__(self, video_capture_path=None):
        rospy.init_node("ip_image_pub", anonymous=True)
        if video_capture_path:
            self.vid = cv2.VideoCapture(video_capture_path)
        else:
            self.get_camera()
            self.vid = cv2.VideoCapture(self.device_num)
        self.counter=0
        self.br = CvBridge()
        rate = rospy.Rate(10)
        self.pub = rospy.Publisher(f'/mrt/camera{self.id}/image_raw', Image, queue_size=10)
        while not rospy.is_shutdown():
            ret, frame = self.vid.read()
            if not ret:
                print("ret == False")
            # print(ret)
            frame=cv2.resize(frame,(256,144))
            # cv2.imshow("frame", frame)
            # cv2.waitKey(10)
            ros_img = self.br.cv2_to_imgmsg(frame,header=Header(seq=self.counter,stamp=rospy.Time.now(),frame_id=FRAME))
            self.pub.publish(ros_img)
            rate.sleep()
            self.counter+=1
        self.vid.release()
        # cv2.destroyAllWindows()

    def get_camera(self):
        self.name = rospy.get_param("~camera_name", "usb-046d_C270_HD_WEBCAM_EA2B8C60-video-index0")
        self.id = rospy.get_param("~camera_id", 1)
        DEFAULT_CAMERA_NAME = "/dev/v4l/by-id/" + self.name
        print(DEFAULT_CAMERA_NAME)
        self.device_num = 0
        if os.path.exists(DEFAULT_CAMERA_NAME):
            device_path = os.path.realpath(DEFAULT_CAMERA_NAME)
            device_re = re.compile("\/dev\/video(\d+)")
            info = device_re.match(device_path)
            if info:
                self.device_num = int(info.group(1))
                print("Using default video capture device on /dev/video"\
                      + str(self.device_num))




if __name__ == "__main__":
    x = ImagePublisher()
