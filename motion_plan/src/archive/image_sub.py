#!/usr/bin/env python
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com

# Import the necessary libraries
import rospy  # Python library for ROS
from sensor_msgs.msg import Image  # Image is the message type
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library


def callback(data):

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)
    cv2.imwrite("/home/khush/Downloads/frame6.jpg", current_frame)
    # Display image
    cv2.imshow("camera", current_frame)

    cv2.waitKey(1)
    # template = cv2.cvtColor(template, cv2.COLOR_BGR2GRAY)
    # ret1, template_bin = cv2.threshold(template, 120, 255, cv.THRESH_BINARY)

    # frame_grayscale = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
    # ret2, frame_img_bin= cv2.threshold(frame_grayscale, 120, 255, cv2.THRESH_BINARY)
    # #apply canny edge detection to the image
    # edges = cv2.Canny(frame_img_bin,50,150,apertureSize = 3)
    # #show what the image looks like after the application of previous functions
    # cv2.imshow("canny'd image", edges)
    # cv2.waitKey(0)


def receive_message():

    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node("video_sub_py", anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber("/mrt/camera1/image_raw", Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv2.destroyAllWindows()


if __name__ == "__main__":
    receive_message()
