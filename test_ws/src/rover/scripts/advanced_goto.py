#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
simple_goto.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to arm and takeoff in Copter and how to navigate to points using Vehicle.simple_goto.

Full documentation is provided at http://python.dronekit.io/examples/simple_goto.html
"""

from __future__ import print_function
import time
from coordinates import fun
from dronekit import connect, VehicleMode, LocationGlobalRelative

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = "/dev/ttyACM1"
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

#arm_and_takeoff(-50)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

# Define the start and end points as latitude and longitude coordinates
start_lat = vehicle.location.global_relative_frame.lat
start_lng = vehicle.location.global_relative_frame.lon
end_lat = 33.6428588
end_lng = -117.8418627

arr = fun(start_lat ,start_lng,end_lat ,end_lng)

#arr = [[start_lat, start_lng],[33.643259,-117.841198]]
i=1
print(arr)
reached = 0
if arr[0][0] == start_lat:
	print("both are same")
else:
	print("false")
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
