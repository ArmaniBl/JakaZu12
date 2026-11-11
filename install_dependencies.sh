#!/usr/bin/env bash
set -euo pipefail

if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required. Install it on Ubuntu 24.04 first." >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  ros-jazzy-control-msgs \
  ros-jazzy-moveit \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rviz2 \
  ros-jazzy-xacro \
  ros-jazzy-ur-description \
  ros-jazzy-joint-state-topic-hardware-interface

echo "Dependencies installed. Run ./build.sh next."
