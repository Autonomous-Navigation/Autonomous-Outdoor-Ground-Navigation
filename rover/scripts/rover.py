#!/usr/bin/env python
# -*- coding: utf-8 -*-


from __future__ import print_function
import time
from coordinates_walking import fun
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



# Print location information for `vehicle` in all frames (default printer)
print ("Global Location: %s" % vehicle.location.global_frame)
print ("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
print ("Local Location: %s" % vehicle.location.local_frame)    #NED

# Print altitudes in the different frames (see class definitions for other available information)
print ("Altitude (global frame): %s" % vehicle.location.global_frame.alt)
print ("Altitude (global relative frame): %s" % vehicle.location.global_relative_frame.alt)
print ("Altitude (NED frame): %s" % vehicle.location.local_frame.down)