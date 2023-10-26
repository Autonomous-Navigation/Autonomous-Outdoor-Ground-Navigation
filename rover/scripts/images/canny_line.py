import cv2
import numpy as np

# Load the Canny edge image
canny_image = cv2.imread('image20231019113926.jpg', cv2.IMREAD_GRAYSCALE)

image = canny_image
img_gray = image
img_gray = cv2.medianBlur(img_gray, 5)
edges = cv2.Laplacian(img_gray, cv2.CV_8U, ksize=15)
ret,mask =cv2.threshold(edges,100,255,cv2.THRESH_BINARY_INV)
image2 = cv2.bitwise_and(image, image, mask=mask)
image2 = cv2.medianBlur(image2, 9)  # this
cv2.imshow("Mask", mask)

# Apply the Hough Line Transform to detect lines in the Canny image
lines = cv2.HoughLines(canny_image, 1, np.pi / 180, threshold=200)

# Create a copy of the original image to draw the lines on
output_image = np.zeros_like(canny_image)
print(lines)
# Draw the detected lines on the output image
if lines is not None:
    for rho, theta in lines[:, 0]:
        a = np.cos(theta)
        b = np.sin(theta)
        x0 = a * rho
        y0 = b * rho
        x1 = int(x0 + 1000 * (-b))
        y1 = int(y0 + 1000 * (a))
        x2 = int(x0 - 1000 * (-b))
        y2 = int(y0 - 1000 * (a))
        cv2.line(output_image, (x1, y1), (x2, y2), 255, 2)

# Display the output image with detected lines
cv2.imshow('Detected Lines', output_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
