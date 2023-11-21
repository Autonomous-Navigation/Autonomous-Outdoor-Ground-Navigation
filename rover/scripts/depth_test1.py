import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Set the height threshold (you may need to adjust this based on your environment)
height_threshold = 0.2  # Adjust this value according to your needs
gradient_threshold = 100  # Adjust this value according to your needs
kernel_size = 1  # Adjust this size based on the noise characteristics
iterations = 1   # Adjust this based on the desired smoothing effect

def depth_image_callback(msg):
    bridge = CvBridge()
    depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Apply height thresholding to detect the curb
    curb_mask = depth_image > height_threshold

    # Apply a Sobel filter for edge detection
    sobel_x = cv2.Sobel(curb_mask.astype(np.uint8) * 255, cv2.CV_64F, 1, 0, ksize=5)
    sobel_y = cv2.Sobel(curb_mask.astype(np.uint8) * 255, cv2.CV_64F, 0, 1, ksize=5)
    gradient_magnitude = np.sqrt(sobel_x**2 + sobel_y**2)

    # Apply gradient thresholding to detect the curb edges
    curb_edges = gradient_magnitude > gradient_threshold

    # Apply morphological operations to filter out noise and enhance lines
    
    # Convert the binary mask to BGR format for display
    curb_edges_bgr = cv2.cvtColor(curb_edges.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)

    # Display the detected curb edges
    cv2.imshow("Curb Detection (Gradient Threshold)", curb_edges_bgr)
    cv2.waitKey(1)

def main():
    rospy.init_node("curb_gradient_detector")
    rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_image_callback)

    # Initialize the OpenCV window for displaying the curb detection
    cv2.namedWindow("Curb Detection (Gradient Threshold)", cv2.WINDOW_NORMAL)
    
    rospy.spin()

if __name__ == "__main__":
    main()

