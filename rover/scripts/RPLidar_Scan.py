#!/usr/bin/env python
# -*- coding: utf-8 -*-

from rplidar import RPLidar
import matplotlib.pyplot as plot
import numpy as np
import time
import RPLFunctions as rpl
import random
import math
import subprocess                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               
import rospy
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation
import matplotlib
import argparse
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float64MultiArray

segthreshold = 150


#Distance between points to separate segments (mm)
segthreshold = 150

#Import published topics from ros and check if /scan exists. If not, start it
topics = rospy.get_published_topics()
topics = list(zip(*topics))[0]
if not any(t == '/scan' for t in topics):
    rplProcess = rpl.start()
    time.sleep(3)

#Initialize listener node 
rospy.init_node('rplidar_scan')
#rpl.listener()

distance_pub = rospy.Publisher('nearest_obstacle_distance', Float64MultiArray, queue_size=1)
delay = rospy.Rate(1)


#Program loops until user exits with CTRL-C
while not rospy.is_shutdown():  
    #Import Scans from ROS topic
    scanvals = rpl.getScan()
    first_index = None
    second_index = None
    third_index = None
    fourth_index = None
    
    for i, sample in enumerate(scanvals):
        if sample[1] > -160:
            first_index = i
            break
    for i, sample in enumerate(scanvals):
        if sample[1] > -140:
            second_index = i
            break
    for i, sample in enumerate(scanvals):
        if sample[1] > 140:
            third_index = i
            break
    for i, sample in enumerate(scanvals):
        if sample[1] > 160:
            fourth_index = i
            break

    left_portion = scanvals[first_index:second_index,:]
    right_portion = scanvals[third_index:fourth_index,:]
    middle_portion = np.concatenate((scanvals[:first_index,:], scanvals[fourth_index:,:]), 0)

    # left_portion = scanvals[-150:-160,:]
    # right_portion = scanvals[-160:,:]
    # middle_portion = np.concatenate((scanvals[:first_index,:], scanvals[third_index:,:]), 0)

    print("Detecting middle obstacle")
    middle_obstacle_distance = float('inf')
    for sample in middle_portion:
        if sample[2] < middle_obstacle_distance:
            middle_obstacle_distance = sample[2]
    if middle_obstacle_distance == float('inf'):
        middle_obstacle_distance = 25000
    print("Middle Obstacle Distance is: ")
    print(middle_obstacle_distance)

    print("Detecting left obstacle")
    left_obstacle_distance = float('inf')
    for sample in left_portion:
        if sample[2] < left_obstacle_distance:
            left_obstacle_distance = sample[2]
    if left_obstacle_distance == float('inf'):
        left_obstacle_distance = 25000
    print("Left Obstacle Distance is: ")
    print(left_obstacle_distance)

    print("Detecting right obstacle")
    right_obstacle_distance = float('inf')
    for sample in right_portion:
        if sample[2] < right_obstacle_distance:
            right_obstacle_distance = sample[2]
    if right_obstacle_distance == float('inf'):
        right_obstacle_distance = 25000
    print("Right Obstacle Distance is: ")
    print(right_obstacle_distance)

    obstacle_distance = Float64MultiArray()
    obstacle_distance.data = [left_obstacle_distance, middle_obstacle_distance, right_obstacle_distance]
    if middle_obstacle_distance != float('inf'):
        distance_pub.publish(obstacle_distance)
	
    delay.sleep()





