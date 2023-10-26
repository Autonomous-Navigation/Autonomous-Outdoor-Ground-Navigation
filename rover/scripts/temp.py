#!/usr/bin/env python

import rospy
from std_msgs.msg import Image  # Import the appropriate message type

def callback(data):
    rospy.loginfo("Received: %s", data.data)  # Print the received message
    # You can perform further processing here if needed

def listener():
    rospy.init_node('listener', anonymous=True)  # Initialize your ROS node
    rospy.Subscriber('/camera/color/image_raw', String, callback)  # Subscribe to the desired topic
    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass

