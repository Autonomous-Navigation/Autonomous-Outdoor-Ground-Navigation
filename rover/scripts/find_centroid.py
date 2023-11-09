#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32  # Import Int32 message type

def image_callback(msg, offset_publisher):
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return

    # Convert the image to grayscale
    print(                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          )
    print(cv_image.shape)
    grayscale_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # Filter out masks labeled as 3 or 4 (road and vegetation)
    cropped_mask = (grayscale_image == 0) #| (grayscale_image == 3) | (grayscale_image == 4)
    #print(road_mask.shape)
#    cropped_mask = road_mask[-8:, :]
    # Find the centroid of the filtered mask
    M = cv2.moments(np.uint8(cropped_mask))
    if M["m00"] != 0:
        centroid_x = int(M["m10"] / M["m00"])
        centroid_y = int(M["m01"] / M["m00"])
        rospy.loginfo("Centroid: ({}, {})".format(centroid_x, centroid_y))
        # Calculate the offset from the image center
        image_center_x = cv_image.shape[1] // 2
        
        offset = centroid_x - image_center_x
        print(offset)
        if offset > 1:
            rospy.loginfo("Turn right")
        elif offset < -1:
            rospy.loginfo("Turn left")
	else:
	    rospy.loginfo("Go Straight")

        # Publish the offset to the /segnet_direction topic
        offset_publisher.publish(offset)

def semantic_segmentation_listener():
    rospy.init_node('semantic_segmentation_listener', anonymous=True)
    
    # Create a publisher for the offset
    offset_publisher = rospy.Publisher('/segnet_direction', Int32, queue_size=1)
    
    rospy.Subscriber('/segnet/class_mask', Image, image_callback, offset_publisher)
    rospy.spin()

if __name__ == '__main__':
    try:
        semantic_segmentation_listener()
    except rospy.ROSInterruptException:
        pass
