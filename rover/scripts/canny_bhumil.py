import cv2
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import time
bridge = CvBridge()
edges_image = None
class_mask_image = None
i=0
j=0
image_count=0
def edges_callback(msg):
	print("in edge callback")
	global edges_image
	global class_mask_image
	global image_count
	global i
	if i!=5:
	        i=i+1
	else:
		cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
		print(cv_image.shape)
                cv2.imshow("Overlay Image", cv_image)
		cv2.waitKey(1)

	        '''gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
	        lol = cv2.Canny(gray, 100, 200)  # Adjust threshold values as needed
	        edges_image = cv2.cvtColor(lol,cv2.COLOR_GRAY2RGB)
	        cv2.imwrite("Image" + str(image_count) + ".jpg", edges_image)
	        image_count=+1
	        print("/camera/color/img_raw size: ")
	        print(cv_image.shape)
	        #edges_image = bridge.imgmsg_to_cv2(edges, desired_encoding="passthrough")
	        #cv2.imshow("Edges", edges_image)
		cv2.waitKey(1)
		if class_mask_image is not None:
	            alpha = 0.8  # Adjust the transparency as needed
	            overlay_image = cv2.addWeighted(edges_image, 1 - alpha, class_mask_image, alpha, 0)
		i=0'''

def class_mask_callback(msg):
        global class_mask_image
	print("in class mask callback")
	global j
	if j!=2:
		j=j+1
	else:
		class_mask_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
		print(class_mask_image.shape)
	        #cv2.imshow("Class_mask", class_mask_image)
		cv2.waitKey(1)
		j=0


def main():
    rospy.init_node('image_subscriber', anonymous=True)
    print("in main")

    # Replace 'your_image_topic' with the name of the ROS topic you want to subscribe to
    #rospy.Subscriber('/video_source/raw', Image, edges_callback)
#    rospy.Subscriber('/camera/color/image_raw', Image, edges_callback)

    # Subscribe to the class mask topic
    rospy.Subscriber('/segnet/class_mask', Image, class_mask_callback)

    # Create a window for displaying the overlay image
    #cv2.namedWindow("Overlay Image", cv2.WINDOW_NORMAL)

    rospy.spin()
    cv2.destroyAllWindows() 

if __name__ == '__main__':
    image_count=0
    main()
