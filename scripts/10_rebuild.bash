#!/bin/bash

cd /home/navigation_ws

source /opt/ros/humble/setup.bash

read -p "Do you wish to perform a clean installation? [y/n] " yn
case $yn in
    [Yy]* ) bash /home/navigation_ws/scripts/01_clear_workspace.bash;; # remove any files from a previous build
    [Nn]* ) ;;
    * ) ;;
esac

colcon build

source /home/navigation_ws/install/local_setup.bash
