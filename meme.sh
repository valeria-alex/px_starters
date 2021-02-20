#!/bin/bash


export PX4_HOME_LAT=55.057883   
export PX4_HOME_LON=10.569678

source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo

#roslaunch gui:=false px4 posix_sitl.launch 
roslaunch px4 posix_sitl.launch 

#check python Qground, pitch pid values
