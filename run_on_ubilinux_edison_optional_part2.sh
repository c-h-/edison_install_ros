#!/bin/bash

DEBUG=false
original_dir="~"
user="root"

RED='\033[0;31m'
NC='\033[0m' # No Color


# handles command exit erroring
on_error () {
	RESULT=$?
	if [ $RESULT -eq 0 ]; then
		echo "Continuing..."
	else
		printf "$RED ERROR: Command failed. Fix the error and rerun the script. ${NC}"
		echo ""
		cd original_dir # return to starting location
		exit
	fi
}

########
# Sets up tutorials, common messages, and android sensor subscriber.
########

# Add the tutorials
cd ~/ros_catkin_ws/src && catkin_create_pkg beginner_tutorials std_msgs rospy roscpp || on_error
cd beginner_tutorials || on_error
mkdir -p scripts || on_error

# Add Android subscriber here
cd ~/ros_catkin_ws/src/beginner_tutorials/scripts || on_error
wget https://raw.githubusercontent.com/c-h-/android_subscriber/master/android_subscriber.py

# Add common messages
cd ~/ros_catkin_ws/src || on_error
rosws set common_msgs --git https://github.com/ros/common_msgs.git || on_error
rosws update common_msgs
cd ~/ros_catkin_ws
# below line is unnecessary if catkin_make (at the end) runs
# catkin_make common_msgs

# make everything
catkin_make

source ~/ros_catkin_ws/devel/setup.bash
echo "source ~/ros_catkin_ws/devel/setup.bash" >> ~/.bashrc