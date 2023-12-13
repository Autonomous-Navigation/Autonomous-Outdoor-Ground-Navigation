#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Float64MultiArray, Int32MultiArray
import sys, select, termios, tty
rospy.init_node('demo_advanced', anonymous=True)

pub = rospy.Publisher("/krishnaaaa", String, queue_size=1)
delay = rospy.Rate(1)


# Function to read keyboard input
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    return key


rospy.loginfo("Press 's' to publish 'start' and 'n' to publish 'stop'. Press Ctrl+C to exit.")

try:
    while not rospy.is_shutdown():
        user_input = getKey()
        if user_input == 's' or user_input == 'n':
            rospy.loginfo("Publishing '%s' to /krishnaaaaa topic.", user_input)
            pub.publish(user_input)
        else:
            pub.publish('none')
        delay.sleep()

except Exception as e:
    print(e)

finally:
    pass

