import realsense2_camera as rs
import cv2
import os

rs = rs.RealsenseCamera()

while True:
    ret,bgr_frame, depth_frame = rs.get_frame_stream()
    distance = depth_frame[100,100]
    print(depth_frame)
    cv2.imshow("Depth Frame", depth_frame)
    key = cv2.waitKey(1)
    if key == 27:
            break
