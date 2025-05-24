#!/bin/bash

echo 'Clearing old build files'
rm -rf /home/navigation_ws/build/ /home/navigation_ws/build_isolated/ /home/navigation_ws/devel /home/navigation_ws/devel_isolated/ /home/navigation_ws/install /home/navigation_ws/install_isolated/ /home/navigation_ws/log/
# potentially unset environment variables