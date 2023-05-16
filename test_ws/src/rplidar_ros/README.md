RPLIDAR ROS package
=====================================================================

ROS node and test application for RPLIDAR

Visit following Website for more details about RPLIDAR:

rplidar roswiki: http://wiki.ros.org/rplidar

rplidar HomePage:   http://www.slamtec.com/en/Lidar

rplidar SDK: https://github.com/Slamtec/rplidar_sdk

rplidar Tutorial:  https://github.com/robopeak/rplidar_ros/wiki

How to build rplidar ros package
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build rplidarNode and rplidarNodeClient

How to run rplidar ros package
=====================================================================
There're two ways to run rplidar ros package

I. Run rplidar node and view in the rviz
------------------------------------------------------------
```bash
roslaunch rplidar_ros view_rplidar.launch (for RPLIDAR A1/A2)
```
```bash
roslaunch rplidar_ros view_rplidar_a3.launch (for RPLIDAR A3)
``` 
```bash
roslaunch rplidar_ros view_rplidar_s1.launch (for RPLIDAR S1)
``` 
```bash 
roslaunch rplidar_ros view_rplidar_s2.launch (for RPLIDAR S2)
``` 
```bash 
roslaunch rplidar_ros view_rplidar_s2e.launch (for RPLIDAR S2E)
``` 
```bash
roslaunch rplidar_ros view_rplidar_t1.launch (for SLAMTEC LPX T Serials)  
``` 
You should see rplidar's scan result in the rviz.

II. Run rplidar node and view using test application
------------------------------------------------------------
```bash
roslaunch rplidar_ros rplidar.launch (for RPLIDAR A1/A2)
```
```bash
roslaunch rplidar_ros rplidar_a3.launch (for RPLIDAR A3)
```
```bash 
roslaunch rplidar_ros rplidar_s1.launch (for RPLIDAR S1)
```
```bash 
roslaunch rplidar_ros rplidar_s2.launch (for RPLIDAR S2)
```
```bash
roslaunch rplidar_ros rplidar_s2e.launch (for RPLIDAR S2E)
```
```bash 
roslaunch rplidar_ros rplidar_t1.launch (for SLAMTEC LPX T Serials)  
```
rosrun rplidar_ros rplidarNodeClient

You should see rplidar's scan result in the console

Notice: the different is serial_baudrate between A1/A2<115200> , A3/S1<256000> and S2<1000000>

RPLidar frame
=====================================================================
RPLidar frame must be broadcasted according to picture shown in rplidar-frame.png
