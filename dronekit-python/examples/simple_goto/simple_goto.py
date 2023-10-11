
#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
from coordinates import fun

import time
from dronekit import connect, VehicleMode, LocationGlobalRelative


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = "/dev/ttyACM0" #args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


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


arm_and_takeoff(-5)

print("Set default/target airspeed to 3")
vehicle.airspeed = 3

start_lat =33.643420
start_lng =-117.840839
end_lat = 33.642420
end_lng =-117.841902

arr = fun(start_lat, start_lng, end_lat, end_lng)

for pts in arr:
	print("Going towards first point for 30 seconds ...")
	point1 = LocationGlobalRelative(pts[0],pts[1], 0)
	vehicle.simple_goto(point1)
	
	# sleep so we can see the change in map
	#time.sleep(30)
	
	#print("Going towards second point for 30 seconds (groundspeed set to 10 m/s) ...")
	#point2 = LocationGlobalRelative(-35.363244, 149.168801, 20)
	#vehicle.simple_goto(point2, groundspeed=10)
	
	# sleep so we can see the change in map
time.sleep(5)
	
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

# Close vehicle object before exiting script
print("Close vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl:
    sitl.stop()
