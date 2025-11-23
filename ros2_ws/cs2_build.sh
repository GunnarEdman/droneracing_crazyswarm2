#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Update package lists and install missing dependencies
apt-get update -qq
rosdep install --from-paths src --ignore-src -y

# Clean previous build artifacts and run the build
cd /ws
# rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release