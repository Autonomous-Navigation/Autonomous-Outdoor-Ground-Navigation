#!/usr/bin/env python
# -*- coding: utf-8 -*-

import pyfiglet
import os
import rospy
from std_msgs.msg import String, Float64MultiArray, Int32MultiArray
import sys, select, termios, tty

# Initialize ROS node
rospy.init_node('demo', anonymous=True)

# Global variable to store the user input
user_input = None

def waypoint_distance_callback(data):
    distance = data.data[0]
    print("Distance from the next waypoint: "+str(distance) + "\n")

def direction_graphics_callback(data):
    os.system('clear')

    camera=data.data[0]
    lidar=data.data[1]
    rover=data.data[2]
    text = "SS  L   R"
    ascii_art = pyfiglet.figlet_format(text, font="block")
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(ascii_art)
    
    
    temp_text=""

    if(camera == 1):
        temp_text=temp_text+"            -> "
    elif(camera == 3):
        temp_text=temp_text+" <-            "
    else:
        temp_text=temp_text+"                    "
    temp_text=temp_text+"|"

    if(lidar == 1):
        temp_text=temp_text+"            -> "
    elif(lidar == 3):
        temp_text=temp_text+" <-            "
    else:
        temp_text=temp_text+"                    "
    temp_text=temp_text+"|"

    if(rover == 1):
        temp_text=temp_text+"            -> "
    elif(rover == 3):
        temp_text=temp_text+" <-            "
    elif(rover == 0):
        temp_text=temp_text+"       ^       "
    else:
        temp_text=temp_text+"                    "

    ascii_art = pyfiglet.figlet_format(temp_text,  font="small", width=100)
    print(ascii_art)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)


# Publisher for the "/krishnaaaaa" topic
pub = rospy.Publisher("/krishnaaaa", String, queue_size=10)
rospy.Subscriber("waypoint_distance", Float64MultiArray, waypoint_distance_callback,queue_size=1)
rospy.Subscriber("direction_graphics", Int32MultiArray, direction_graphics_callback, queue_size=1)


# Function to read keyboard input
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
    return key

# Main loop
original_settings = termios.tcgetattr(sys.stdin)

# Main loop
settings = termios.tcgetattr(sys.stdin)
rospy.loginfo("Press 's' to publish 'start' and 'n' to publish 'stop'. Press Ctrl+C to exit.")

try:
    while not rospy.is_shutdown():
        user_input = getKey()
        user_input = 'a'
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
               
        if user_input == 's' or user_input == 'n':
            rospy.loginfo("Publishing '%s' to /krishnaaaaa topic.", user_input)
            pub.publish(user_input)
        else:
            exit()

except Exception as e:
    print(e)

finally:
    pass

