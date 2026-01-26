#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/../ros2_ws"

source /opt/ros/humble/setup.bash || true

# Build your workspace
colcon build --symlink-install
source install/setup.bash

# XRCE agent for SITL (UDP)
# PX4 SITL usually connects on UDP port 8888 by default (configurable in PX4).
microxrceagent udp4 -p 8888 &

# Launch ROS graph
ros2 launch uav_bringup uav_sitl.launch.py
