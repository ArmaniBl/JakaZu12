#!/usr/bin/env bash
# Source this file in each WSL terminal used for this workspace.
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# Prefer an explicitly selected distribution, otherwise use the installed ROS.
ros_distro=${ROS_DISTRO:-}
if [[ -z $ros_distro ]]; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then ros_distro=jazzy
  elif [[ -f /opt/ros/humble/setup.bash ]]; then ros_distro=humble
  fi
fi
if [[ ! -f /opt/ros/${ros_distro}/setup.bash ]]; then
  echo "Install ROS 2 Humble (Ubuntu 22.04) or Jazzy (Ubuntu 24.04)." >&2
  return 1
fi
source "/opt/ros/${ros_distro}/setup.bash"
if [[ -f "$workspace_dir/install/local_setup.bash" ]]; then
  source "$workspace_dir/install/local_setup.bash"
fi
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}
# Retain the existing Windows/WSL Fast DDS peer profile, if present.
if [[ -f "$HOME/.ros/fastdds.xml" ]]; then
  export FASTDDS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds.xml"
  export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/.ros/fastdds.xml"
fi
