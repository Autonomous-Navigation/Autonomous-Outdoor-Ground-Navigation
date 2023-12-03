#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String
import time

# Initialize the ROS node
rospy.init_node('simple_goto_node', anonymous=True)

# Create a publisher for the "xyz" topic
pub = rospy.Publisher('simple_goto_topic', String, queue_size=1)

# Set the loop rate to 1/15 Hz (every 15 seconds)
rate = rospy.Rate(1/15.0)

while not rospy.is_shutdown():
    # Create a String message
    message = String()
    message.data = "Hello"

    # Publish the message on the "xyz" topic
    pub.publish(message)

    # Sleep to maintain the desired publishing rate
    rate.sleep()