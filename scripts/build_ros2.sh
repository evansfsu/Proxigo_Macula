#!/usr/bin/env bash
set -e

# build ROS2 workspace in this repo
source /opt/ros/humble/setup.bash

cd "$(dirname "$0")/.."
cd ros2_ws

# Clean build optional (uncomment if you want)
# rm -rf build install log

colcon build --symlink-install
echo ""
echo "Build done. Now run:"
echo "  source /opt/ros/humble/setup.bash"
echo "  source $(pwd)/install/setup.bash"
