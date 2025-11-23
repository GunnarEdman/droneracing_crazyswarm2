# Setup script for ROS 2 workspace, clones repos from crazyswarm2.repos
#!/bin/bash
set -e
echo "Setting up workspace..."

# 1. Install the tool that reads .repos files
if ! command -v vcs &> /dev/null; then
    sudo apt-get update && sudo apt-get install -y python3-vcstool
fi

# 2. Read the list and download repos into src/
vcs import < crazyswarm2.repos

echo "Done! Code downloaded."