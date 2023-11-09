#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def depth_image_callback(data):
    try:
        bridge = CvBridge()
        depth_image = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    except Exception as e:
        print(e)
        return

    # Apply basic thresholding to classify obstacles and free areas
    # You should fine-tune these values and apply more advanced techniques in a real-world scenario.
    threshold_value = 0  # Depth value in meters
    max_depth = 0.1  # Maximum depth value in meters

    # Threshold the depth image to classify obstacles (values below threshold) and free areas (values above threshold)
    obstacle_mask = (depth_image < threshold_value) & (depth_image <= max_depth)
    free_area_mask = depth_image > max_depth
    # Create a visualization image (for demonstration purposes)
    depth_visualization = np.zeros_like(depth_image, dtype=np.uint8)
    depth_visualization[obstacle_mask] = 0  # Mark obstacles as white
    depth_visualization[free_area_mask] = 255    # Mark free areas as black

    # Display the processed depth image
    cv2.imshow("Processed Depth Image", depth_visualization)
    cv2.waitKey(1)

def main():
    rospy.init_node('depth_image_processing')
    rospy.Subscriber('/camera/depth/image_rect_raw', Image, depth_image_callback)

    rate = rospy.Rate(10)  # Adjust the rate as needed

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

