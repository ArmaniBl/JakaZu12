#!/usr/bin/env bash
# Source this file in each WSL terminal used for this workspace.
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  echo "ROS 2 Jazzy is required in Ubuntu 24.04." >&2
  return 1
fi
# Use local_setup to avoid bringing the old isaac_moveit_ws overlay back in.
source /opt/ros/jazzy/setup.bash
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
