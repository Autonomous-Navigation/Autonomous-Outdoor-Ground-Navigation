#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

def image_callback(data):
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

    # Convert to HSV color space
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # Define a lower and upper range for red color
    lower_red = np.array([0, 1, 200])
    upper_red = np.array([360, 30, 255])
    # Create a mask to extract only red pixels
    mask = cv2.inRange(hsv_image, lower_red, upper_red)

    # Bitwise-AND mask and original image to get red color
    red_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    # Display the red-only image
    cv2.imshow("Red Image", red_image)
    cv2.waitKey(3)

def main():
    rospy.init_node('red_color_extraction_node', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()

