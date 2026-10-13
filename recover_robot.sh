#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
robot_ip=${1:-10.5.5.100}

if [[ ! $robot_ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  echo "Usage: $0 [robot-ip]" >&2
  exit 2
fi

if pgrep -f '/jaka_planner/moveit_server' >/dev/null 2>&1; then
  echo "ERROR: moveit_server is running. Stop run_real.sh with Ctrl+C first." >&2
  exit 1
fi

cat <<'EOF'
This will STOP the controller program, abort motion, recover collision state,
clear controller errors, power on, and enable the robot.

It will not send a trajectory, but the robot can energize its joints.
Clear the work area and keep the emergency stop within reach.
EOF

read -r -p "Type RECOVER to continue: " confirmation
if [[ $confirmation != RECOVER ]]; then
  echo "Canceled."
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "$workspace_dir/install/setup.bash"
set -u

exec python3 "$workspace_dir/log_run.py" recovery -- \
  ros2 run jaka_planner recover_robot "$robot_ip"
