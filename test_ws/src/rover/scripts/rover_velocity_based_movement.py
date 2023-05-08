##########DEPENDENCIES#############
from dronekit import connect, VehicleMode,LocationGlobalRelative,APIException
import time
import rospy
from std_msgs.msg import Int16
import socket
#import exceptions
import math
import argparse
from pymavlink import mavutil


connection_string = '/dev/ttyACM0'
#########FUNCTIONS#################

def connectMyCopter():

	#parser = argparse.ArgumentParser(description='commands')
	#parser.add_argument('--connect')
	#args = parser.parse_args()

	#connection_string = args.connect
	vehicle = connect(connection_string,baud=57600,wait_ready=True)

	return vehicle

def arm():
	while vehicle.is_armable!=True:
		print("Waiting for vehicle to become armable.")
		time.sleep(1)
	print("Vehicle is now armable")

	vehicle.mode = VehicleMode("GUIDED")

	while vehicle.mode!='GUIDED':
		print("Waiting for drone to enter GUIDED flight mode")
		time.sleep(1)
	print("Vehicle now in GUIDED MODE. Have fun!!")

	vehicle.armed = True
	while vehicle.armed==False:
		print("Waiting for vehicle to become armed.")
		time.sleep(1)
	print("Vehicle is now armed.")

	return None

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

stop = 1

def calculation_for_stopage(data):
	print(data.data)
	global stop
	if data.data < 500:
#		send_global_ned_velocity(0,0,0)
		stop = 1
		#time.sleep(1)
	else:
		stop = 0
		

##########MAIN EXECUTABLE###########

vehicle = connectMyCopter()
print("The Pixhawk is connected with Jetson Nano")
arm()

rospy.init_node('velocity')
rospy.Subscriber("nearest_obstacle_distance", Int16, calculation_for_stopage)



counter=0
while True:
	print(stop)
	if stop == 0:
	        send_local_ned_velocity(1,0,0) 
	        print("run")
	        #time.sleep(1)
	else:
		send_global_ned_velocity(0,0,0)
		print("stop")
		#time.sleep(1)
        counter = counter + 1







'''
counter=0

while counter <5:
        send_global_ned_velocity(0,0,0)
        counter = counter + 1


counter=0
while counter < 2:
        send_local_ned_velocity(1,1,0)
        print("Turning to the right")
        time.sleep(1)
#        send_local_ned_velocity(1,-1,0)
#        print("Turning to the left")
        counter = counter + 1
counter=0

while counter < 5:
        send_global_ned_velocity(0,0,0)
        time.sleep(2)
        counter = counter+1
counter=0

while counter < 5:
        send_global_ned_velocity(0,0,0)
        print("Moving TRUE NORTH")
        time.sleep(1)
        #send_global_ned_velocity(-1,0,0)
        #print("MOVING TRUE SOUTH")
        #time.sleep(3)
        #send_global_ned_velocity(0,1,0)
        #print("MOVING TRUE EAST")
        #time.sleep(3)
        #send_global_ned_velocity(0,-1,0)
        #print("MOVING TRUE WEST")
        #time.sleep(3)
        counter=5'''


