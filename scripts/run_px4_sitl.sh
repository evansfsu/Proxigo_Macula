#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"

if [ ! -d "$PX4_DIR" ]; then
  echo "PX4_DIR not found: $PX4_DIR"
  echo "Clone PX4 first: git clone https://github.com/PX4/PX4-Autopilot.git $PX4_DIR"
  exit 1
fi

# Typical SITL build
cd "$PX4_DIR"
make px4_sitl_default

echo "PX4 SITL running. In another terminal run: ./scripts/run_ros_graph.sh"
