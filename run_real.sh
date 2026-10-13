#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
robot_ip=${1:-10.5.5.100}
servo_lpf_cutoff_hz=${2:-0.5}

if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ./build.sh first." >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u

exec python3 "$workspace_dir/log_run.py" real -- \
  ros2 launch jaka_zu12_moveit_config real_robot.launch.py \
  ip:="$robot_ip" \
  auto_enable:=false \
  servo_lpf_cutoff_hz:="$servo_lpf_cutoff_hz"
