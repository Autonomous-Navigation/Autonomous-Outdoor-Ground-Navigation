#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray, Bool
from geopy.distance import geodesic

last_distnace = float('-inf')



def gps_callback(data):
    # Ensure the received data has the expected format (array of two elements)


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

    if(curr_distance < last_distance):
        last_distnace = curr_distance
    else:
        last_distance = float('-inf')
        next_waypoint=True
        waypoint_pub.publish(Bool(next_waypoint))
    


def main():
    print("in1")
    
    rospy.init_node('gps_data_receiver')    
    rospy.Subscriber('gps_data', Float64MultiArray, gps_callback)
    waypoint_pub = rospy.Publisher('next_waypoint', Bool, queue_size=1)
    
    print("in5")


   
    rospy.spin()

if __name__ == '__main__':
    main()
