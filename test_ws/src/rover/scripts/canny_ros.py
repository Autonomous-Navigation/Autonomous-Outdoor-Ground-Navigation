import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

# Initialize the CvBridge
bridge = CvBridge()
edges_image = None
class_mask_image = None

new_width = 424
new_height = 240


def class_mask_callback(msg):
    global class_mask_image
    try:
        # Convert the ROS Image message to an OpenCV image
        class_mask_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
	# Resize the image to the new dimensions
        #class_mask_image = cv2.resize(class_mask_image, (edges_image.shape[1], edges_image.shape[0]))
	cv2.imshow("Class_mask", class_mask_image)
    except Exception as e:
        rospy.logerr(e)

def edges_callback(msg):
    global edges_image
    global class_mask_image
    try:
        # Convert the ROS Image message to an OpenCV image
        # Convert the ROS Image message to an OpenCV image
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Convert the frame to grayscale (Canny requires grayscale input)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Apply Canny edge detection
        edges_image = cv2.Canny(gray, 100, 200)  # Adjust threshold values as needed
        edges_image = cv2.cvtColor(edges_image,cv2.COLOR_GRAY2RGB)
	#edges_image = bridge.imgmsg_to_cv2(edges, desired_encoding="passthrough")
	cv2.imshow("Edges", edges_image)
	if class_mask_image is not None:
            # Overlay the class mask on top of the edges image
            alpha = 0.1  # Adjust the transparency as needed
            print("In edge")
            print(edges_image.shape)
            print(class_mask_image.shape)
            print(type(edges_image))
            print(type(class_mask_image))
            overlay_image = cv2.addWeighted(edges_image, 1 - alpha, class_mask_image, alpha, 0)
	    cv2.namedWindow("Overlay Image", cv2.WINDOW_NORMAL)
            # Display the overlay image
            cv2.imshow("Overlay Image", overlay_image)
    except Exception as e:
        rospy.logerr(e)

'''def image_callback(msg):
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
'''

def main():
    rospy.init_node('image_subscriber', anonymous=True)
    print("in")
    # Replace 'your_image_topic' with the name of the ROS topic you want to subscribe to
    rospy.Subscriber('/video_source/raw', Image, edges_callback)

    # Subscribe to the class mask topic
    rospy.Subscriber('/segnet/overlay', Image, class_mask_callback)

    # Create a window for displaying the overlay image
    cv2.namedWindow("Overlay Image", cv2.WINDOW_NORMAL)

    #while not rospy.is_shutdown():
    '''if edges_image is not None and class_mask_image is not None:
            # Overlay the class mask on top of the edges image
            alpha = 0.1  # Adjust the transparency as needed
	    print("In main")
	    print(edges_image.shape)
	    print(class_mask_image.shape)
	    print(type(edges_image))
	    print(type(class_mask_image))
            overlay_image = cv2.addWeighted(edges_image, 1 - alpha, class_mask_image, alpha, 0)
	
            # Display the overlay image
            #cv2.imshow("Overlay Image", overlay_image)

        cv2.waitKey(1)'''
    # Initialize the OpenCV window (optional, only if you want to display the frames)
    cv2.namedWindow("Original Image", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Edges", cv2.WINDOW_NORMAL)

    rospy.spin()

if __name__ == '__main__':
    main()


