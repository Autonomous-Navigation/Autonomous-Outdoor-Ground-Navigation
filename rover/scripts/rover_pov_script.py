#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import csv
import time

width = 512
height = 256

count=0
# Create a NumPy array filled with zeros
rover_pov = np.zeros((height, width), dtype=int)

class RoverPOVPublisher:
    def __init__(self):
        rospy.init_node('rover_pov_publisher', anonymous=True)

        # Initialize your subscribers
        self.depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw', Image, self.depth_callback)
        #self.mask_sub = rospy.Subscriber('/segnet/class_mask', Image, self.class_mask_callback)
        #self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        #self.compressed_depth_sub = rospy.Subscriber('/camera/depth/image_rect_raw/compressed', CompressedImage, self.compressed_depth_callback)
        # Initialize your publisher
        #self.rover_pov_pub = rospy.Publisher('/rover_pov', Int32MultiArray, queue_size=1)
        self.bridge = CvBridge()
        self.delay = rospy.Rate(1)

    def compressed_depth_callback(self, data):
        cv_image = self.bridge.compressed_imgmsg_to_cv2(data, "passthrough")
        image_array = np.array(cv_image)
        with open('compressed_depth_data.csv', 'w') as file:
                writer = csv.writer(file)
                writer.writerows(image_array)
        print(image_array.shape)
        time.sleep(10)

    def depth_callback(self, data):
        global rover_pov
        print("In depth callback")
        cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")  # Convert the ROS Image message to an OpenCV image
        image_array = np.array(cv_image, dtype=np.uint16)
        '''
        new_width = 512
        new_height = 256
        resized_image = cv2.resize(cv_image, (new_width, new_height))
        image_array = np.array(cv_image, dtype=np.uint16)
        image_array_roi = image_array[image_array.shape[0]/2:,:]
        
        # with open('depth_data.csv', 'w') as file:
        #     writer = csv.writer(file)
        #     writer.writerows(image_array)
        print(image_array_roi.shape)
        
        '''
        #depth_image_smooth = cv2.GaussianBlur(image_array_roi, (5, 5), 0)
        depth_image_gray = (image_array * 255.0 / image_array.max()).astype(np.uint8)
        #print(depth_image_gray.max())
        # Process the depth image as needed

        threshold_min = 50  # Adjust these values
        threshold_max = 150
        
        edges = cv2.Canny(depth_image_gray, threshold_min, threshold_max)

        '''

        # Apply Hough Line Transform
        threshold=50
        lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold)

        # Draw the detected lines on a copy of the original image
        line_image = np.copy(edges)

        if lines is not None:
            for line in lines:
                rho, theta = line[0]
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 2)

        # Display the image with detected lines
        
        
        contours, _ = cv2.findContours(line_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Define a minimum contour area or length threshold (adjust as needed)
        min_contour_area = 100  # You can adjust this value

        # Filter out small contours
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > min_contour_area]

        # Create an image with only the filtered contours
        filtered_image = np.zeros_like(edges)
        cv2.drawContours(filtered_image, filtered_contours, -1, (255, 255, 255), thickness=cv2.FILLED)




        '''
        # Display the processed depth image
        #cv2.imshow('Processed Depth Image', filtered_image)
        cv2.imshow('gr', edges)
        #cv2.imshow("Lines Detected", cv_image)
        cv2.waitKey(1)
        '''
        # Get the shape of the input depth array
        rows, cols = image_array.shape

        # Create an output array to store the shar#p changes
        #sharp_changes = np.zeros((rows, cols), dtype=np.int32)
        sharp_changes = np.empty((rows, cols))
        threshold = 15
        ng_threshold = -15
        
        # Iterate through the depth array
        for i in range(1, rows - 1):
            for j in range(1, cols - 1):
                center_depth = image_array[i, j]
                neighbors = [image_array[i-1, j-1], image_array[i-1, j], image_array[i-1, j+1],
                            image_array[i, j-1], image_array[i, j+1],
                            image_array[i+1, j-1], image_array[i+1, j], image_array[i+1, j+1]]

                # Check if the difference between the center depth and any neighbor exceeds the threshold
                #print(neighbors)
                depth=False
                for neighbor in neighbors:
                    print(center_depth)
                    print(neighbor)
                    diff = abs(center_depth - neighbor)
                    if diff > threshold:
                        sharp_changes[i, j] = 255
                        depth=True
                        break
                #if any(abs(center_depth - neighbor) > threshold for neighbor in neighbors):
                #    sharp_changes[i, j] = 255
                if depth==True:
                    sharp_changes[i,j] = 0
        cv2.imshow("lol", sharp_changes)
        cv2.waitKey(1)
        count=0
        #update rover_pov here'''

    def class_mask_callback(self, data):
        global rover_pov
        global width
        global height
        print("class_mask_callback")
        cv_image = self.bridge.imgmsg_to_cv2(data, "passthrough")  # Convert the ROS Image message to an OpenCV image
        print(cv_image.shape)
        cv_image = cv2.resize(cv_image, (width, height))
        print(cv_image.shape)
        image_array = np.array(cv_image)
        new_array = np.where((image_array == 2) | (image_array == 3) | (image_array == 4), 0, 1)
        print(new_array.shape)
        #update rover_pov using new_image here

    def scan_callback(self, data):
        global rover_pov
        print("In scan_callback")
        angle_distance = self.getScan(data)
        print(angle_distance)
        #update rover_pov here
    
    def getScan(self, data):
        #waits for a pubished message from the /scan topic - (topic, data type, timeout (sec))
        scan = data 

        #yields range data (float32[]) -- ranges is an array which gives the distance measured for each angle bin
        #Ranges are returned in meters -- convert to mm 
        ranges = np.array(scan.ranges)*1000

        #np.array(dataype?, copy(bool): True - object is copied/False - copy only made if array returns copy, subok(bool): True - subclasses will be passed through/False - forced to be base-class array, ndmn(int): specifies minimum number of dimensions resulting array should have)
        #[:] -- colon returns all indicies in the array
        ranges = np.array(ranges[:], copy=False, subok=True, ndmin=2).T

        #yields the intensity data (float32[]) -- intensities of 0 return no value, as the intensity increases the quality of the reading improves
        intensities = np.array(scan.intensities)
        intensities = np.array(intensities[:], copy=False, subok=True, ndmin=2).T
        
        #converts the minimum/maximum angle (angle at which the first/last scan point is taken) from the scan from radians to degrees (float32)
        
        anglemin = np.rad2deg(scan.angle_min) 
        anglemax = np.rad2deg(scan.angle_max)

        #yields the angular resolution or the angle between one scan point and the next
        angleincr = np.rad2deg(scan.angle_increment)
        angles = []
    
        #the LaserScan message reports a list of ranges and the angle increment -- calculates the individual angles corresponding to their range values
        for i in range(len(scan.ranges)):
            angles.append(anglemin + angleincr*i)
        angles = np.array(angles[:], copy=False, subok=True, ndmin=2).T
        print("111111")
        #sets up the scanvals variable as a numpy array with [intensities angles ranges]
        scanvals = np.concatenate((intensities,angles,ranges),axis=1)
        #deletes rows where intesnity == 0 
        scanvals = np.delete(scanvals, np.where(scanvals[:,0]==0),axis=0)
        scanvals = np.delete(scanvals, np.where((scanvals[:,1] < 155) & (scanvals[:,1] > -155)), axis=0)
        #scanvals = np.delete(scanvals, np.where(scanvals[:,1] < 175), axis=0)
        #inserts a fourth column of zeros for later use
        scanvals = np.insert(scanvals, 3, 0, axis=1)
        return scanvals

    def run(self):
        while not rospy.is_shutdown():
            #rover_pov_struct = Int32MultiArray()
            #rover_pov_struct.data = rover_pov
            #self.rover_pov_pub.publish(rover_pov_struct)
            self.delay.sleep()

if __name__ == '__main__':
    rover_pov_node = RoverPOVPublisher()
    rover_pov_node.run()
