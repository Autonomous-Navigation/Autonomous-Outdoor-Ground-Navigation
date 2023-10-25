#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2

image_array = cv2.imread("images/image20231019113833original.jpg")
print(image_array.shape)
depth_image_gray = (image_array * 255.0 / image_array.max()).astype(np.uint8)

# Process the depth image as needed
threshold_min = 0.5  # Adjust these values
threshold_max = 1.0


edges = cv2.Canny(depth_image_gray, threshold_min, threshold_max)
print(edges.shape)
lol = cv2.resize(edges, (640, 360))
print(lol.shape)
# Display the processed depth image
cv2.imshow('Processed Depth Image', edges)
cv2.waitKey(0)