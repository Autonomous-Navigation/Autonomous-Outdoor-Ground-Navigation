#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial

def serial_reader():
    rospy.init_node('serial_reader', anonymous=True)

    # Set the serial port and baud rate
    serial_port = '/dev/video2'  # Replace with your serial port
    baud_rate = 9600  # Replace with your baud rate

    # Open the serial port
    ser = serial.Serial(serial_port, baud_rate)

    # Create a ROS publisher
    pub = rospy.Publisher('serial_data', String, queue_size=10)

    rate = rospy.Rate(10)  # 10 Hz

    while not rospy.is_shutdown():
        # Read data from the serial port
        data = ser.readline().strip()

        # Publish the data on the ROS topic
        pub.publish(data)

        # Print the data for debugging
        rospy.loginfo("Received data: %s", data)

        rate.sleep()

    # Close the serial port when done
    ser.close()

if __name__ == '__main__':
    try:
        serial_reader()
    except rospy.ROSInterruptException:
        pass
