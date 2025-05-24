#!/bin/bash

# register source for rosdep
echo 'Registering source for rosdep and updating apt'
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
apt-get update

# DIR="/home/navigation_ws/src/ros2_numpy"
# if [ -d "$DIR" ]; then
#   echo "ros2_numpy already cloned"
# else
#   cd /home/navigation_ws/src
#   git clone https://github.com/Box-Robotics/ros2_numpy.git
#   echo "cloned ros2_numpy"
# fi

# not working because full driver is not working - only adma_ros_driver_msgs and adma_ros2_driver
# DIR="/home/colcon_ws/src/adma_ros_driver"
# if [ -d "$DIR" ]; then
#   echo "adma_ros_driver already installed"
# else
#   cd /home/colcon_ws/src
#   git clone https://github.com/GeneSysElektronik/adma_ros_driver.git
#   echo "cloned adma_ros_driver"
# fi
cd /home/navigation_ws
# source ros 2 distro
echo 'Updating rosdeps'
rosdep update --rosdistro humble
rosdep install --from-path src --ignore-src -r -y

# enforce clean build
# bash /home/navigation_ws/scripts/01_clear_workspace.bash

# build colcon ws
echo 'Building workspace'
source /opt/ros/humble/setup.bash

source /home/navigation_ws/install/local_setup.bash
