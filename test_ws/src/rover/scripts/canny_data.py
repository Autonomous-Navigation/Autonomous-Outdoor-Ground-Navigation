#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
counter=0
import datetime

class ImageProcessorNode:
    def __init__(self):
        print("in init")
        rospy.init_node('image_processor_node', anonymous=True)
        self.bridge = CvBridge()

        # Subscribe to the /camera/color/image_raw topic
        rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        global counter
        print("in callback")
        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            rospy.logerr("CvBridgeError: {0}".format(e))
            return

        # Apply Canny edge detection
        edges = cv2.Canny(cv_image, 100, 200)

        # Save the updated image locally
        if(counter%50==0):
            timestamp = datetime.datetime.now().strftime('%Y%m%d%H%M%S')
            cv2.imwrite('/home/agni/agni_ws/src/rover/scripts/images/image' + timestamp + '.jpg', edges)
            cv2.imwrite('/home/agni/agni_ws/src/rover/scripts/images/image' + timestamp + 'original.jpg', cv_image)
	counter=counter+1
        print(counter)

def main():
    image_processor = ImageProcessorNode()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
