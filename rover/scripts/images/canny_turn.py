import cv2
import numpy as np

# Load the canny edge image
canny_image = cv2.imread('image20231019113846.jpg', cv2.IMREAD_GRAYSCALE)

# Define the region of interest (ROI) where the curb ends are located
# You may need to adjust these values based on the specific image
roi_x = 100  # X-coordinate of the top-left corner of the ROI
roi_y = 100  # Y-coordinate of the top-left corner of the ROI
roi_width = 400  # Width of the ROI
roi_height = 300  # Height of the ROI

# Crop the ROI from the canny edge image
#roi = canny_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
roi=canny_image
# Detect lines using the Hough Line Transform
lines = cv2.HoughLinesP(roi, 1, np.pi / 180, threshold=100, minLineLength=50, maxLineGap=10)

# Create an empty image to draw the curb ends
result_image = np.zeros_like(roi)

# Iterate through the detected lines and draw the vertical lines on the result image
for line in lines:
    x1, y1, x2, y2 = line[0]
    # Check if the line is roughly vertical (you may need to adjust the angle range)
    if abs(x1 - x2) < 5:
        cv2.line(result_image, (x1, y1), (x2, y2), 255, 2)

# Create an empty canvas with the same size as the original image
#output_image = np.zeros_like(canny_image)
output_image=result_image
# Place the result image with curb ends in the original image
#output_image[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width] = result_image

# Display the output image with curb ends
cv2.imshow('Curb Ends', output_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
