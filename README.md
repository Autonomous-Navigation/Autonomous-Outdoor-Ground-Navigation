# Autonomous-Outdoor-Ground-Navigation
We are group of University of California, Irvine graduate students working on designing a robust and reliable navigation system for a pair of rover and drone for our Capstone Project.

### Get stated with Jetson Nano: 
[Step by step installation guide](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

### To make your Jetson ready type below commands in the terminal:
* sudo apt update
* sudo apt update
* sudo apt upgrade
* sudo apt install curl
* sudo apt-get install nano
* sudo apt install python-pip
* sudo apt-get install libxml2
* sudo apt-get install libxslt-dev
* pip install dronekit dronekit-sitl mavproxy

### Installing ROS Melodic:
[Official ROS documentation](http://wiki.ros.org/melodic/Installation/Ubuntu)
* sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
* curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add –
* sudo apt update
* sudo apt install ros-melodic-desktop-full
* echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
* source ~/.bashrc
* sudo apt install python-rosdep python-rosinstall python-rosinstall-generator
* python-wstool build-essential
* sudo apt install python-rosdep
* sudo rosdep init
* rosdep update

### To create a ROS environment in your Jetson Nano
* mkdir -p ~/agni_ws/src
* cd ~/agni_ws/src
* catkin_init_workspace
* cd ..
* catkin_make
* echo "source ~/agni_ws/devel/setup.bash" >> ~/.bashrc
* source ~/.bashrc

### Cloning and using this repository
* Clone this repository: https://github.com/Autonomous-Navigation/Autonomous-Outdoor-Ground-Navigation
* Copy 
    * ros_deep_learning
    * rover
    * rplidar_ros
* into ~/agni_ws/src
* cd ~/agni_ws
* catkin_make


### Debugging some of the problems
* [vision_opencv](https://github.com/ros-perception/vision_opencv/issues/345)
* /opt/ros/melodic/share/cv_bridge/cmake/cv_bridgeConfig.cmake 
* change this line:
* set(_include_dirs "include;/usr/include;/usr/include/opencv") 
* to
* set(_include_dirs "include;/usr/include;/usr/include/opencv4")
* https://jstar0525.tistory.com/119
* sudo apt-get update
* sudo apt-get install ros-melodic-ddynamic-reconfigure


### Installing Intel Realsense D455 Camera dependencies
[Official Intel Documentation](https://dev.intelrealsense.com/docs/nvidia-jetson-tx2-installation?_ga=2.116855366.1736326955.1684966278-1232809164.1684966278)

* sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
* sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo bionic main" -u
* sudo apt-get install librealsense2-utils
* sudo apt-get install librealsense2-dev
* sudo apt-get install ros-melodic-vision-msgs