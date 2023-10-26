#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Initialize a CvBridge
bridge = CvBridge()

def callback(data):
    try:
        # Convert the ROS Image message to a OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")  # Assuming it's a color image (bgr8 encoding)
        
        # Access the pixel values of the image
        pixel_values = np.asarray(cv_image)

        # Print some pixel values (you can iterate through the array as needed)
        print("Pixel values at (0,0):", pixel_values[0, 0])

        # You can add more code to process the pixel values here

    except CvBridgeError as e:
        rospy.logerr(e)

def listener():
    rospy.init_node('image_listener', anonymous=True)
    rospy.Subscriber('/camera/color/image_raw', Image, callback)  # Replace with your image topic
    rospy.spin()

if __name__ == '__main__':
    listener()

