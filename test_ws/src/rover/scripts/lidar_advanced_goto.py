
#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import time
from coordinates import fun
from dronekit import connect, VehicleMode, LocationGlobalRelative
import rospy
from std_msgs.msg import Int16

from dronekit import APIException


import socket
#import exceptions
import math
import argparse
from pymavlink import mavutil

# Set up option parsing to get connection string
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = "/dev/ttyACM0"
sitl = None



# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


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
	print("Sent MAVLink message")
	vehicle.flush()

def reverse(direction):
	# create the CONDITION_YAW command using command_long_encode()
	msg = vehicle.message_factory.command_long_encode(
		0, 0, # target system, target component
		mavutil.mavlink.MAV_CMD_DO_SET_REVERSE, #command
		0, #confirmation
		direction, #Param 1, 0 for forward 1 for backward.
		0,  #Param 2, yaw speed deg/s
		0, #Param 3, Direction -1 ccw, 1 cw
		0, # Param 4, relative offset 1, absolute angle 0
		0,0, 0) # Param 5-7 not used
	vehicle.send_mavlink(msg)
	vehicle.flush()

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
	vehicle.mode = VehicleMode("GUIDED")
	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter MANUAL flight mode")
		time.sleep(1)
	vehicle.channels.overrides = {'2':1400}
	time.sleep(1)
	vehicle.channels.overrides = {'2':1500}

	vehicle.mode = VehicleMode("GUIDED")
	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)

arm_and_takeoff(-50)
stop = 0

def calculation_for_stopage(data):
	global point1
	print(data.data)
	global stop
	if data.data < 500:
		send_global_ned_velocity(0,0,0)
		stop = 1
		#time.sleep(1)
		print("stopped")
	else:
		if stop == 1:
			vehicle.simple_goto(point1)
		stop = 0
		print("go with flow")

rospy.init_node('velocity')
rospy.Subscriber("nearest_obstacle_distance", Int16, calculation_for_stopage)


print("Set default/target airspeed to 3")
vehicle.airspeed = 3

# Define the start and end points as latitude and longitude coordinates
start_lat = 33.6432884
start_lng = -117.8411328
end_lat = 33.643147
end_lng = -117.841397

arr = fun(start_lat ,start_lng,end_lat ,end_lng)

#arr = [[start_lat, start_lng],[33.643259,-117.841198]]
i=1
point1 = None



for pts in arr:
	print(pts)
	print(i)
	print(pts[0])
	print(pts[1])
	print(type(pts[0]))
	print(type(pts[1]))
	point1 = LocationGlobalRelative(float(pts[0]),float(pts[1]), 0)
	reached =0
	vehicle.simple_goto(point1)
	while (((vehicle.location.global_relative_frame.lat - pts[0]) > 0.00001) or ((vehicle.location.global_relative_frame.lon - pts[1]) > 0.0001)):
		print("going to point ", i)
		time.sleep(1)
		print("before if", stop)
		#if stop == 1:
		#	send_global_ned_velocity(0,0,0)
		#	print("stopped2")
	i=i+1
	# sleep so we can see the change in map

#print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
#point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
#vehicle.simple_goto(point2, groundspeed=10)

# sleep so we can see the change in map
#time.sleep(30)

print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
