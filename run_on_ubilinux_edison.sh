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

# do some checks and setup
startup () {
	# get original_dir for recovery
	original_dir="$(pwd)" || on_error

	# check for root
	user="$(whoami)" || on_error
	# echo "${user}"

	# if [ "$user" != "root" ]; then
	if [ "$user" != "root" ] && [ ! DEBUG ]; then
		printf "$RED ERROR: Must be logged in as root to continue. Logged in as $user ${NC}"
		echo ""
		exit
	fi

	# check for ubilinux env
	environment="$(uname -a)"
	# if [ "$environment" != *"ubilinux"* ]; then
	if ! echo "$environment" | grep -q "ubilinux" ; then
		printf "$RED ERROR: Must be run from a ubilinux installation. Current environment is:${NC}"
		echo ""
		printf "$environment"
		echo ""
		exit
	fi

	cd ~ || on_error
}

# 
# http://martinkronberg.com/new-blog/2015/3/18/ros-on-intel-edison-using-ubilinux-and-porting-to-yocto
pre_install_ros () {
	apt-get install screen git || on_error
	printf '%s\n' 'deb http://http.debian.net/debian wheezy-backports main' >> /etc/apt/sources.list || on_error
	## apt-get update || on_error
	apt-get install liblz4-dev || on_error
	sh -c 'echo "deb http://packages.ros.org/ros/ubuntu wheezy main" > /etc/apt/sources.list.d/ros-latest.list' || on_error
	wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | apt-key add - || on_error

	## apt-get update || on_error
	## apt-get upgrade || on_error

	apt-get install python-setuptools python-pip python-yaml python-argparse python-distribute python-docutils python-dateutil python-setuptools python-six python-empy || on_error

	pip install rosdep rosinstall_generator wstool rosinstall || on_error

	# throws error if already init; ignore
	rosdep init

	## rosdep update || on_error

	apt-get install liblz4-dev || on_error

}

# installs Robot Operating System
# http://martinkronberg.com/new-blog/2015/3/18/ros-on-intel-edison-using-ubilinux-and-porting-to-yocto
install_ros () {
	mkdir -p ~/ros_catkin_ws || on_error
	cd ~/ros_catkin_ws || on_error

	# WRITES TO STD_ERR
	rosinstall_generator ros_comm --rosdistro indigo --deps --wet-only --exclude roslisp --tar > indigo-ros_comm-wet.rosinstall || on_error

	wstool init src indigo-ros_comm-wet.rosinstall || on_error

	mkdir -p ~/ros_catkin_ws/external_src || on_error

	apt-get install checkinstall cmake || on_error

	sh -c 'echo "deb-src http://mirrordirector.raspbian.org/raspbian/ testing main contrib non-free rpi" >> /etc/apt/sources.list' || on_error

	apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 9165938D90FDDD2E || on_error

	## apt-get update || on_error

	# These commands fail.
	# apt-get build-dep console-bridge || on_error
	# apt-get source -b console-bridge || on_error
	# dpkg -i libconsole-bridge0.2*.deb libconsole-bridge-dev_*.deb || on_error

	# Run these commands instead to build and install the package
	cd ~/ros_catkin_ws/external_src || on_error
	apt-get install libboost-system-dev libboost-thread-dev || on_error # libboost-dev libboost-program-options-dev libboost-regex-dev libboost-filesystem-dev libtinyxml-dev libbz2-dev liblog4cxx10-dev  || on_error
	git clone https://github.com/ros/console_bridge.git || on_error
	cd console_bridge || on_error
	cmake . || on_error
	checkinstall make install || on_error
	cd ~/ros_catkin_ws || on_error
	
	# IGNORE ERRORS FROM THIS COMMAND
	rosdep install --from-paths src --ignore-src --rosdistro indigo -y -r --os=debian:wheezy

	./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/indigo || on_error

	# Add ROS to path, set to run when bash starts up
	source /opt/ros/indigo/setup.bash || on_error
	echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc

	# Need to merge /usr/local/lib into /usr/lib
	# ignore errors because errors if file exists
	cp -r -n /usr/local/lib/i386-linux-gnu/* /usr/lib/i386-linux-gnu/

	# OPTIONAL, installs beginner tutorials (see http://wiki.ros.org/ROS/Tutorials/CreatingPackage)
	# and android subscriber script 
	# comment out below line to disable this part of the install
	./run_on_ubilinux_edison_optional_part2.sh

	# cleanup a bit
	apt-get autoremove || on_error

	echo "ROS Install Complete!" || on_error
}


# INSTALLATION FLOW
startup || on_error
pre_install_ros || on_error
install_ros || on_error
