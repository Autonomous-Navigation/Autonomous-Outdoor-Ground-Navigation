#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2

image_array = cv2.imread("images/image20231019113833original.jpg")
print(image_array.shape)
depth_image_smooth = cv2.GaussianBlur(image_array, (5, 5), 0)
depth_image_gray = (image_array * 255.0 / image_array.max()).astype(np.uint8)

# Process the depth image as needed
threshold_min = 50  # Adjust these values
threshold_max = 100


edges = cv2.Canny(depth_image_gray, threshold_min, threshold_max)
print(edges.shape)
# Apply Hough Line Transform
threshold=50
lines = cv2.HoughLines(edges, 1, np.pi / 180, threshold)

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





# Display the processed depth image
#cv2.imshow('Processed Depth Image', filtered_image)
#cv2.imshow('depth raw', depth_image_gray)
cv2.imshow("Lines Detected", edges)
cv2.waitKey(1)
count=0






