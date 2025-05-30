#!/bin/bash

cd /home/navigation_ws

source /opt/ros/jazzy/setup.bash

read -p "Do you wish to perform a clean installation? [y/n] " yn
case $yn in
    [Yy]* ) bash /home/navigation_ws/scripts/01_clear_workspace.bash;; # remove any files from a previous build
    [Nn]* ) ;;
    * ) ;;
esac

read -p "Do you wish to perform a symlink installation? [y/n] " yn
case $yn in
    [Yy]* ) colcon build --symlink-install;;
    [Nn]* ) colcon build;;
    * ) ;;
esac

source /home/navigation_ws/install/local_setup.bash
