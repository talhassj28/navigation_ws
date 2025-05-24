#!/bin/bash

cd /home/navigation_ws
# source ros 2 distro
source /opt/ros/humble/setup.bash
# rosdep update --rosdistro humble
# rosdep install --from-path src --ignore-src -r -y
# build colcon ws
# enforce clean build
read -p "Do you wish to perform a clean installation? [y/n] " yn
case $yn in
    [Yy]* ) bash /home/navigation_ws/scripts/01_clear_workspace.bash;; # remove any files from a previous build
    [Nn]* ) ;;
    * ) ;;
esac

colcon build

source /home/navigation_ws/install/local_setup.bash
