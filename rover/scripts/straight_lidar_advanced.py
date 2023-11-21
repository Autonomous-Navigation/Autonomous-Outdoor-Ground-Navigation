#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from coordinate_critical import fun
from coordinates_walking import fun_coordinates_walking
#from gps_distance import distance_calculation
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rospy
from std_msgs.msg import Int16
from std_msgs.msg import Float64MultiArray
from dronekit import APIException
from std_msgs.msg import Int32, Bool  # Import Int32 message type
import socket
import argparse
from pymavlink import mavutil
from argparse import ArgumentParser
# Set up option parsing to get connection string
print("hi")
parser = ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
print("hi")

try:
    connection_string = "/dev/ttyACM0"
    print('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)
except:
    try:
        connection_string = "/dev/ttyACM1"
        print('Connecting to vehicle on: %s' % connection_string)
        vehicle = connect(connection_string, wait_ready=True)
    except:
        try:
            connection_string = "/dev/ttyACM2"
            print('Connecting to vehicle on: %s' % connection_string)
            vehicle = connect(connection_string, wait_ready=True)
        except:
            print("connected to some new port")
sitl = None

#print("in1")
#start_lat = vehicle.location.global_relative_frame.lat #33.645142
#start_lng = vehicle.location.global_relative_frame.lon #-117.842741
start_lat = 33.647583 #33.643789
start_lng =  -117.840745 #-117.840766
end_lat = 33.643524 #33.64331
end_lng = -117.843106 #-117.841095
print("in2")

def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    print(vehicle.mode)


    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
         # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

##Send a velocity command with +x being the heading of the drone.
def send_local_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0,
		0, 0,
		mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
		0b0000111111000111,
		0, 0, 0,
		vx, vy, vz,
		0, 0, 0,
		0, 0)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def reverse(direction):
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, #command
		0, #confirmation
		1, #Param 1, 0 for forward 1 for backward.
		0,  #Param 2, yaw speed deg/s
		0, #Param 3, Direction -1 ccw, 1 cw
		0, # Param 4, relative offset 1, absolute angle 0
		0,0, 0) # Param 5-7 not used
	vehicle.send_mavlink(msg)
#	vehicle.flush()

##Send a velocity command with +x being the heading of the drone.
def send_global_ned_velocity(vx, vy, vz):
	msg = vehicle.message_factory.set_position_target_local_ned_encode(
		0, # time_boot_ms (not used)
		0, 0, # target system, target component
		mavutil.mavlink.MAV_FRAME_LOCAL_NED, #frame
		0b0000111111000111, #type_mask (only speeds enabled)
		0, 0, 0, # x, y, z positions (not used)
		vx, vy, vz, # x, y, z velocity in m/s
		0, 0, 0, #x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
		0, 0) #yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
	vehicle.send_mavlink(msg)
	vehicle.flush()

def backup(): ##rough function to easily reverse without needing to use a GPS navigation based movement
	mode = "GUIDED"
	vehicle.mode = VehicleMode("MANUAL")
	while vehicle.mode!=mode:
		print("Waiting for drone to enter MANUAL flight mode")
		time.sleep(1)
	vehicle.channels.overrides = {'2':1400}
	time.sleep(2)
	vehicle.channels.overrides = {'2':1500}
	time.sleep(2)
	vehicle.mode = VehicleMode(mode)
	while vehicle.mode!=mode:
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)

#arm_and_takeoff(-50) #uncommented this 
stop = 0
obstacle = 0
point1 = None
stop_flag=False
def calculation_for_stopage(obstacle_data):
	global point1
	global stop, obstacle 
	
	if obstacle_data.data[1] < 1500:
		obstacle = 1
		if max(obstacle_data.data[0] , obstacle_data.data[2]) < 1000:
			send_local_ned_velocity(0,0,0)
			print("Stop, all ways are blocked from LiDAR")
		elif obstacle_data.data[0] < obstacle_data.data[2]:
			print("turn right from LiDAr")
			send_local_ned_velocity(1,1,0)
		else:
			print("turn left from LiDAR")
			send_local_ned_velocity(1,-1,0)
'''	else:
		if obstacle == 1:
			vehicle.simple_goto(point1)
		obstacle = 0
'''

def calculation_for_direction_using_segnet(segnet_direction):
	global point1,stop_flag	
	print(segnet_direction.data)
	if segnet_direction.data == -20:
		stop_flag=True
		print("back up")
		send_local_ned_velocity(-1,0,0)
		time.sleep(2)
	elif segnet_direction.data < -4:
		stop_flag=True
		print("left")
		send_local_ned_velocity(1,-0.5,0)
		time.sleep(2)
	elif segnet_direction.data > 4:
		stop_flag=True
		print("right")
		send_local_ned_velocity(1,0.5,0)
		time.sleep(2)
	else:
		stop_flag=False
		print("keep straight")

next_waypoint_flag=False


def next_waypoint_callback(msg):
	print("next_wp")
	#global next_waypoint_flag
	send_local_ned_velocity(1,1,0)
	time.sleep(1)
	#next_waypoint_flag=True

from geopy.distance import geodesic

def distance_calculation(pt1_lat, pt1_lon, pt2_lat, pt2_lon):
    pt1_co = (pt1_lat, pt1_lon)
    pt2_co = (pt2_lat, pt2_lon)
    distance = geodesic(pt1_co, pt2_co).meters
    return distance

arr = fun(start_lat ,start_lng,end_lat ,end_lng)
print(len(arr))
print(arr)

def processarr():
    length=len(arr)
    for i in range(length - 1):
        current_point = arr[i]
        next_point = arr[i + 1]

        latitude, longitude = current_point[0]
        distance = current_point[2]

        if distance > 40:
            print( longitude,latitude, next_point[0][1], next_point[0][0])
            new_pt=fun_coordinates_walking( longitude,latitude, next_point[0][1], next_point[0][0])
        temp=i+1
        for pt in new_pt:
            arr.insert(temp,[[pt[0],pt[1]],2,30])
            temp=temp+1
            length=length+1
            i=i+1
			

current_target = None
processarr()
#arr = [[33.643322, -117.841024], [33.643488, -117.840712],[33.643561, -117.840563]]
print(len(arr))
print(arr)
i=1

def my_callback(event):
	global current_target
	print("calling simple_goto every 15 seconds")
	vehicle.simple_goto(current_target)
	time.sleep(2)

rospy.init_node('velocity')
#rospy.Subscriber("nearest_obstacle_distance", Float64MultiArray, calculation_for_stopage)
rospy.Subscriber("segnet_direction", Int32, calculation_for_direction_using_segnet,queue_size=1)
#rospy.Subscriber("next_waypoint", Bool, next_waypoint_callback)
pub = rospy.Publisher('gps_data', Float64MultiArray, queue_size=1)
rate = rospy.Rate(1)  # 1 Hz
timer = rospy.Timer(rospy.Duration(15), my_callback)


for critical_pts in arr:
	pts = critical_pts[0]
	point1 = LocationGlobalRelative(float(pts[0]),float(pts[1]), 0)
	reached =0
	vehicle.simple_goto(point1)
	time.sleep(5)

	distance = critical_pts[2]
	if distance > 30:


	gps_msg = Float64MultiArray()

	while (distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,pts[0],pts[1])>5):
		print(distance_calculation(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,pts[0],pts[1]))
		gps_msg.data = [
		vehicle.location.global_relative_frame.lat,  # Replace with your actual GPS data source for latitude
		vehicle.location.global_relative_frame.lon,  # Replace with your actual GPS data source for longitude
		pts[0],
		pts[1],
		]
		print("in while loop1")

		pub.publish(gps_msg)
		print("in while loop2")
		send_local_ned_velocity(1,0,0)
		time.sleep(1)
	print("reached the point")

	# sleep so we can see the change in map

# sleep so we can see the change in map
time.sleep(5)

print("Returning to Launch")
#vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
