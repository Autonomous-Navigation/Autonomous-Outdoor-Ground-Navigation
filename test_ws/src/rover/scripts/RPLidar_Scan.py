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

# plot.ion()

segthreshold = 150

#Definte Landmark Positions
'''cornerA=[0,0]
cornerB=[0,280.5]
cornerC=[164.5,280.5]
cornerD=[164.5,0]'''

#Define current Robot Position
# RPLpose=[20,20,45]

#Distance between points to separate segments (mm)
segthreshold = 150

#list of colors to draw segments with
# colors = ['b','g','c','m','y','k','w']


#Import published topics from ros and check if /scan exists. If not, start it
topics = rospy.get_published_topics()
topics = list(zip(*topics))[0]
if not any(t == '/scan' for t in topics):
    rplProcess = rpl.start()
    time.sleep(3)

#Initialize listener node 
rpl.listener()

distance_pub = rospy.Publisher('nearest_obstacle_distance', Float64MultiArray, queue_size=1)
delay = rospy.Rate(1)


#Program loops until user exits with CTRL-C
while True:
    #Get time at beginning of sript
    start_time = time.time()
    
    # plot.clf()
    
    #Import Scans from ROS topic
    scanvals = rpl.getScan()
    print(scanvals)
    print(scanvals.shape)
    
    first_index = None
    second_index = None
    third_index = None
    
    for i, sample in enumerate(scanvals):
        if sample[1] > -175:
	    first_index = i
	    break
    for i, sample in enumerate(scanvals):
	if sample[1] > -155:
	    second_index = i
	    break
    for i, sample in enumerate(scanvals):
        if sample[1] > 175:
	    third_index = i
	    break

    left_portion = scanvals[first_index:second_index,:]
    right_portion = scanvals[second_index:third_index,:]
    middle_portion = np.concatenate((scanvals[:first_index,:], scanvals[third_index:,:]), 0)

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
	
    
    
    '''x = (scanvals[:,2])*(np.cos(np.deg2rad(scanvals[:,1])))
    y = (scanvals[:,2])*(np.sin(np.deg2rad(scanvals[:,1])))
    
    #segment scan points based on segthreshold
    scanvals = rpl.segment(scanvals, segthreshold)
    #scanvals = rpl.splitSeg(scanvals, x, y)
    #Plot the lidar scan one segment at a time, alternating colors
    j=0
    for i in range(int(scanvals[-1][3])):
        plot.figure(1)
        scat = plot.scatter((x[(scanvals[:,3]==i)]), (y[(scanvals[:,3]==i)]), marker='*', c=colors[j])
        plot.axis('equal')
        j+=1
        j%=len(colors)
        
    #For every segment, run line checker and store them in lines
    lines = np.empty([0,3]) 
    linecount = 0
    alldata = np.empty([0,3])    
    for k in range(int(scanvals[-1][3])):
        if np.size(np.where(scanvals[:,3]==k)) > 10:
            xV=x[scanvals[:,3]==k]
            yV=y[scanvals[:,3]==k]
            seg = np.full_like(xV, k)
            data=np.column_stack((xV, yV, seg))
            line = rpl.getLine(data)[1]
            line = np.reshape(line,(1,2))
            line = np.column_stack((line, np.full_like(range(len(line)),seg[0])))
            lines = np.vstack((lines,line))
            alldata = np.append(alldata, data, axis=0)
            
    #Take the average of similar lines
    i=0
    segs = np.unique(lines[:,2])   
    avglines = np.empty([0,3])
    for i in range(len((segs))):
        avg = np.average(lines[lines[:,2]==segs[i]], axis=0)
        avglines = np.append(avglines, np.reshape(avg, (1,3)),axis=0)
    
    #Plot avglines ontop of each segment
    i=0    
    for i in range(len(avglines)):
        if np.any(alldata[:,2] == avglines[i,2]):
            plot.figure(1)
            plot.plot([np.min(alldata[(np.where(alldata[:,2]==avglines[i,2])[0]),0]),np.max(alldata[(np.where(alldata[:,2]==avglines[i,2])[0]),0])],
                      np.polyval(avglines[i,(0,1)],
                                  [np.min(alldata[(np.where(alldata[:,2]==avglines[i,2])[0]),0]),
                                  np.max(alldata[(np.where(alldata[:,2]==avglines[i,2])[0]),0])]),
                    'r')

    plot.draw()'''

    #Print time it took for this loop to complete
    print("Time: ", time.time() - start_time, "seconds")
    delay.sleep()
    #pause matplotlib, gives it time to draw and allows "animation"
    #plot.pause(0.001)





