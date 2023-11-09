#!/usr/bin/env python
# -*- coding: utf-8 -*-

#This script is to read depth data from the CSV file and convert it into opencv image

import numpy as np
import cv2

np_array = np.genfromtxt('depth_data.csv', delimiter=',')
unique=np.unique(np_array)
print(unique)
cv2.imshow("lol", np_array)
cv2.waitKey(0)

print(np_array)