#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyfiglet
import os
import rospy
from std_msgs.msg import String, Float64MultiArray, Int32MultiArray
import sys, select, termios, tty

# Initialize ROS node
rospy.init_node('demo_con', anonymous=True)

pub = rospy.Publisher("/direction_graphics", Int32MultiArray, queue_size=1)
delay = rospy.Rate(1)


while not rospy.is_shutdown():
    direction_msg = Int32MultiArray()
    direction_msg.data = [3, 1, 3]
    pub.publish(direction_msg)
    delay.sleep()