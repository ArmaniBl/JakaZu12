#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required. Install it on Ubuntu 22.04 first." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  ros-humble-control-msgs \
  ros-humble-moveit \
  ros-humble-ros2-controllers \
  ros-humble-rviz2 \
  ros-humble-xacro

echo "Dependencies installed. Run ./build.sh next."
