#!/usr/bin/env python3
# license removed for brevity

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
from imutils import paths
import numpy as np
import imutils
def find_marker(image):
	# convert the image to grayscale, blur it, and detect edges
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (5, 5), 0)
	edged = cv2.Canny(gray, 35, 125)
	# find the contours in the edged image and keep the largest one;
	# we'll assume that this is our piece of paper in the image
	cnts = cv2.findContours(edged.copy(), cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
	cnts = imutils.grab_contours(cnts)
	c = max(cnts, key = cv2.contourArea)
	# compute the bounding box of the of the paper region and return it
	return cv2.minAreaRect(c)

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

# initialize the known distance from the camera to the object, which
# in this case is 24 inches
KNOWN_DISTANCE = 24.0
# initialize the known object width, which in this case, the piece of
# paper is 12 inches wide
KNOWN_WIDTH = 5.0

def showImage(img):
    cv2.imshow('image', img)
    cv2.waitKey(1)
    
def process_image(msg):
    try:
        
        bridge = CvBridge()
        orig = bridge.imgmsg_to_cv2(msg, "bgr8")
        drawImg = orig 
        #showImage(drawImg)
        image = cv2.imread("arrow_qt1.jpeg")
        marker = find_marker(image)
        focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH
        cap=orig
        for imagePath in sorted(paths.list_images("arrow_sample/imgs")):
        	img = cap#cv2.imread(imagePath)
        	img=cv2.resize(img,(720,720))
        	#print (imagePath)
        	#showImage(img)
        	marker = find_marker(img)
        	inches = distance_to_camera(KNOWN_WIDTH, focalLength, marker[1][0])
        	box = cv2.cv.BoxPoints(marker) if imutils.is_cv2() else cv2.boxPoints(marker)
        	box = np.int0(box)
        	cv2.drawContours(img, [box], -1, (0, 255, 0), 2)
        	img=cv2.putText(img, "%.2fft" % (inches / 12),(img.shape[1] - 200,img.shape[0] -20),cv2.FONT_HERSHEY_SIMPLEX,2.0, (0, 255, 0), 1, 200)
        	cv2.resizeWindow("image2", 100, 100)
        	cv2.imshow("image2", img)
        	cv2.waitKey(1)
    except Exception as err:
        print ("err")
      
    


def start_node():
    rospy.init_node('detect_pump')
    rospy.loginfo('detect_pump node started')
    rospy.Subscriber("/arm_sensor/camera/image_raw", Image, process_image)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
