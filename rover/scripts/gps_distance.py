#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Bool
from geopy.distance import geodesic

last_distance = float('inf')
count=0

def distance_calculation(pt1_lat, pt1_lon, pt2_lat, pt2_lon):
    pt1_co = (pt1_lat, pt1_lon)
    pt2_co = (pt2_lat, pt2_lon)
    distance = geodesic(pt1_co, pt2_co).meters

    return distance

def gps_callback(data):
    # Ensure the received data has the expected format (array of two elements)
    global last_distance

    # Extract latitude and longitude from the received data
    latitude, longitude = data.data[0], data.data[1]

    # Example coordinates to calculate the distance to
    target_latitude = data.data[2]  # Replace with your target latitude
    target_longitude = data.data[3]  # Replace with your target longitude

    # Calculate the distance using geodesic distance calculation
    current_coordinates = (latitude, longitude)
    target_coordinates = (target_latitude, target_longitude)
    curr_distance = geodesic(current_coordinates, target_coordinates).meters

    next_waypoint=False
    global count
    print("curr_distance")
    print(curr_distance)
    print(last_distance)
    if(curr_distance - last_distance <= 0.2):
        last_distance = curr_distance
	print("last disyance is current now")
    else:
	if count>=4:
	        last_distance = float('inf')
	        next_waypoint=True
	        waypoint_pub.publish(Bool(next_waypoint))
		count=0
		print("published")
	else:
		count=count+1
    



print("in1")

rospy.init_node('gps_data_receiver')    
rospy.Subscriber('gps_data', Float64MultiArray, gps_callback)
waypoint_pub = rospy.Publisher('next_waypoint', Bool, queue_size=1)

print("in5")



rospy.spin()



