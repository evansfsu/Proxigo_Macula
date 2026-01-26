#!/usr/bin/env bash
set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
  git curl \
  python3-colcon-common-extensions python3-rosdep \
  micro-xrce-dds-agent || true

sudo rosdep init 2>/dev/null || true
rosdep update
echo "Done."
