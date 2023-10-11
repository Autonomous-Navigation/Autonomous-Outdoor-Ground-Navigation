import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Initialize the CvBridge
bridge = CvBridge()

def image_callback(msg):
    try:
        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        print("in1")

        # Convert the frame to grayscale (Canny requires grayscale input)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply Canny edge detection
        edges = cv2.Canny(gray, 100, 200)  # Adjust threshold values as needed

        # Display the original frame and the edges
        cv2.imshow("Original Image", cv_image)
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr(e)

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    print("in")
    # Replace 'your_image_topic' with the name of the ROS topic you want to subscribe to
    rospy.Subscriber('/video_source/raw', Image, image_callback)

    # Initialize the OpenCV window (optional, only if you want to display the frames)
    cv2.namedWindow("Original Image", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Edges", cv2.WINDOW_NORMAL)

    rospy.spin()

if __name__ == '__main__':
    main()

