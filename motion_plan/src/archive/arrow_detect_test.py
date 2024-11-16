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

import numpy as np
import cv2 as cv
import os


def detect_arrow(frame, template, check=False):
    (tH, tW) = template.shape[:2]
    found = None
    direction = None
    template = cv.cvtColor(template, cv.COLOR_BGR2GRAY)
    ret1, template_bin = cv.threshold(template, 80, 255, cv.THRESH_BINARY)

    frame_grayscale = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    ret2, frame_img_bin = cv.threshold(frame_grayscale, 80, 255, cv.THRESH_BINARY)

    # detect edges in the resized, grayscale image and apply template
    # matching to find the template in the image
    for tem_scale in np.linspace(0.2, 1.0, 10)[::-1]:
        r_tem = tem_scale

        template_resize = cv.resize(
            template_bin,
            dsize=(
                int(template_bin.shape[1] * tem_scale),
                int(template_bin.shape[0] * tem_scale),
            ),
            interpolation=cv.INTER_LINEAR,
        )
        if frame.shape[0] < tH * tem_scale or frame.shape[1] < tW * tem_scale:
            continue

        result = cv.matchTemplate(frame_img_bin, template_resize, cv.TM_SQDIFF)
        (minVal, _, minLoc, _) = cv.minMaxLoc(result)
        minVal = minVal / (template_resize.size)

        # check to see if the iteration should be visualized
        # draw a bounding box around the detected region
        """
        clone = np.dstack([edged, edged, edged])
        cv.rectangle(clone, (maxLoc[0], maxLoc[1]),
                     (maxLoc[0] + int(tW * tem_scale), maxLoc[1] + int(tH * tem_scale)), (0, 0, 255), 2)
        cv.imshow("checking", clone)
        cv.waitKey(0)
        """
        if found is None or minVal < found[0]:
            found = (minVal, minLoc, r_tem)
            direction = 1

    for tem_scale in np.linspace(0.2, 1.0, 10)[::-1]:
        r_tem = tem_scale

        template_resize = cv.resize(
            template_bin,
            dsize=(
                int(template_bin.shape[1] * tem_scale),
                int(template_bin.shape[0] * tem_scale),
            ),
            interpolation=cv.INTER_LINEAR,
        )
        if frame.shape[0] < tH * tem_scale or frame.shape[1] < tW * tem_scale:
            continue
        template_inversion = template_resize[:, ::-1]

        result = cv.matchTemplate(frame_img_bin, template_inversion, cv.TM_SQDIFF)
        (minVal, _, minLoc, _) = cv.minMaxLoc(result)
        minVal = minVal / (template_resize.size)
        # check to see if the iteration should be visualized
        # draw a bounding box around the detected region
        """
        clone = np.dstack([edged, edged, edged])
        cv.rectangle(clone, (maxLoc[0], maxLoc[1]),
                     (maxLoc[0] + int(tW * tem_scale), maxLoc[1] + int(tH * tem_scale)), (0, 0, 255), 2)
        """

        if found is None or minVal < found[0]:

            found = (minVal, minLoc, r_tem)
            direction = -1

    # unpack the bookkeeping variable and compute the (x, y) coordinates
    # of the bounding box based on the resized ratio

    (_, minLoc, r_tem) = found

    (startX, startY) = (int(minLoc[0]), int(minLoc[1]))
    (endX, endY) = (int(minLoc[0] + tW * r_tem), int(minLoc[1] + tH * r_tem))
    # draw a bounding box around the detected result and display the image
    cv.rectangle(frame, (startX, startY), (endX, endY), (0, 0, 255), 2)

    return frame, direction


def callback(data):

    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    rospy.loginfo("receiving video frame")

    # Convert ROS Image message to OpenCV image
    frame = br.imgmsg_to_cv2(data)
    template = cv.imread("/home/phoenix6017/Downloads/arrow.jpeg")
    # Display image
    # cv2.imshow("camera", frame)
    output, direction = detect_arrow(frame, template)
    if direction == 1:
        direction = "Right"
    else:
        direction = "Left"
    # font
    font = cv.FONT_HERSHEY_SIMPLEX

    # org
    org = (50, 50)

    # fontScale
    fontScale = 1

    # Blue color in BGR
    color = (255, 0, 0)

    # Line thickness of 2 px
    thickness = 2

    output = cv.putText(
        output, direction, org, font, fontScale, color, thickness, cv.LINE_AA
    )

    cv.imshow("Arrow", output)
    cv.waitKey(1)


def receive_message():

    # Tells rospy the name of the node.
    # Anonymous = True makes sure the node has a unique name. Random
    # numbers are added to the end of the name.
    rospy.init_node("video_sub_py", anonymous=True)

    # Node is subscribing to the video_frames topic
    rospy.Subscriber("/mrt/camera1/image_raw", Image, callback, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    # Close down the video stream when done
    cv.destroyAllWindows()


if __name__ == "__main__":
    receive_message()
