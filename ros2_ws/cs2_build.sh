#!/bin/bash
set -e

# Script that setups cs2 by first installing ros dependenceis, then building it
cd /ws

echo "\nSourcing ROS 2"
source /opt/ros/jazzy/setup.bash

echo "Updating package lists and installing missing dependencies"
apt-get update
rosdep install --from-paths src --ignore-src -y

echo "Building ros2 workspace"
# rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release