#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32  # Import Int32 message type
import time

time = time.time()

def overlay_callback(msg):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Convert the image to grayscale
    cv2.imwrite("overlay{}.jpg".format(time), cv_image)
    print("overlay image stored")
    

def image_callback(msg, offset_publisher):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Convert the image to grayscale
    cv2.imwrite("classmask{}.jpg".format(time), cv_image)
    print("image stored")
    grayscale_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Filter out masks labeled as 3 or 4 (road and vegetation)
    #cropped_mask = (grayscale_image == 0)
    whole_cropped_mask = (grayscale_image == 3) | (grayscale_image == 4)
    scaled_matrix = np.uint8(whole_cropped_mask*255)
    image = cv2.resize(scaled_matrix,  (512, 256), interpolation=cv2.INTER_NEAREST)
    cv2.imwrite("o_1{}.jpg".format(time), scaled_matrix)
    cropped_mask = whole_cropped_mask[3:7,:]
    print(cropped_mask)
    height, width = len(cropped_mask), len(cropped_mask[0])
    #result_image = [[0] * width for _ in range(height)]
    print("between")

    for i in range(1, height - 1):
        for j in range(1, width - 1):
            # Check 8 neighbors
            if all(cropped_mask[i][j + y] == 1 for y in [-1, 1]):
                cropped_mask[i][j] = 1
    # Find the centroid of the filtered mask
    print(cropped_mask)

    is_filled_with_1 = (cropped_mask == 1).all()
    is_filled_with_0 = (cropped_mask == 0).all()
    cropped_mask_left = cropped_mask[:, :cropped_mask.shape[1] // 2]
    cropped_mask_right = cropped_mask[:, cropped_mask.shape[1] // 2:]
    print("cropped_mask_left")
    print(cropped_mask_left)
    print("cropped_mask_right")
    print(cropped_mask_right)
    sum_left = np.sum(cropped_mask_left)
    sum_right = np.sum(cropped_mask_right)
    print("sum_left")
    print(sum_left)
    print("sum_right")
    print(sum_right)

    if is_filled_with_1:
        offset=0
    elif is_filled_with_0:
        offset=-20
    else:
        offset=sum_right-sum_left
        
#        offset = centroid_x - image_center_x
    print(offset)
    # Publish the offset to the /segnet_direction topic
    offset_publisher.publish(offset)
    

def semantic_segmentation_listener():
    rospy.init_node('semantic_segmentation_listener', anonymous=True)
    
    # Create a publisher for the offset
    offset_publisher = rospy.Publisher('/segnet_direction', Int32, queue_size=1)
    
    rospy.Subscriber('/segnet/class_mask', Image, image_callback, offset_publisher)
    rospy.Subscriber('/segnet/overlay', Image, overlay_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        semantic_segmentation_listener()
    except rospy.ROSInterruptException:
        pass





'''    M = cv2.moments(np.uint8(cropped_mask))
    if M["m00"] != 0:
        centroid_x = int(M["m10"] / M["m00"])
        centroid_y = int(M["m01"] / M["m00"])
        rospy.loginfo("Centroid: ({}, {})".format(centroid_x, centroid_y))
        # Calculate the offset from the image center
        image_center_x = cv_image.shape[1] // 2'''
