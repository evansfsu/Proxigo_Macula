# uav-gps-denied (starter)

ROS 2 Humble + PX4 + uXRCE-DDS bridge baseline.

## Quickstart (SITL on dev PC)
1) Install ROS 2 Humble + colcon
2) Run:
   ./scripts/run_px4_sitl.sh
   ./scripts/run_ros_graph.sh

## Hardware (TELEM2 -> Orin USB via FTDI)
- Configure PX4 params:
  MAV_1_CONFIG=0
  UXRCE_DDS_CFG=102
  SER_TEL2_BAUD=921600
- Run:
  ros2 launch uav_bringup uav_hw_telem2.launch.py dev:=/dev/ttyUSB0 baud:=921600
