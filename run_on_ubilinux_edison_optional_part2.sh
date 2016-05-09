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
# Start optional setup
# Sets up tutorials and android sensor subscriber.
# Not necessary to finish ROS installation
# ~~~~~Add github url to master project~~~~~
########
# Add the tutorials
cd ~/ros_catkin_ws/src && catkin_create_pkg beginner_tutorials std_msgs rospy roscpp || on_error
cd beginner_tutorials || on_error
mkdir -p scripts || on_error
# wget subscriber here
cd ~/ros_catkin_ws || on_error
########
# End optional setup
########