0#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import time

def run_command_in_terminal(command, terminal_title):
    formatted_command = "gnome-terminal --tab --title='{}' --command='bash -c \"{}\"'".format(terminal_title, command)
    subprocess.Popen(formatted_command, shell=True)

if __name__ == "__main__":
    run_command_in_terminal("roslaunch realsense2_camera rs_camera.launch", "Camera Initialize")
    time.sleep(10)
    run_command_in_terminal("roslaunch rplidar_ros rplidar.launch", "LiDAR Initialize")
    time.sleep(10)
    run_command_in_terminal("roslaunch ros_deep_learning segnet.ros1.launch model_name:=fcn-resnet18-cityscapes-512x256 input_width:=424 input_height:=240 output:=display://0", "Semantic Segmentation")
    time.sleep(25)
    run_command_in_terminal("python find_centroid.py", "Find Centroid")
    time.sleep(5)
    run_command_in_terminal("python RPLidar_Scan.py", "RPLScan")
    time.sleep(5)
    #run_command_in_terminal("python rover_pov_script.py", "POV Depth")
    time.sleep(5)
    #run_command_in_terminal("rviz","LIDAR Rviz")
    time.sleep(5)
    run_command_in_terminal("python demo_advanced.py", "Demo Advanced")
