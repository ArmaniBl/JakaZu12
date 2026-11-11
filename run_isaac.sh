#!/usr/bin/env bash
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$workspace_dir/setup_isaac.bash"
if [[ ! -d "$workspace_dir/install/ur10_isaac_moveit_config" ]]; then
  echo "Run ./build.sh first." >&2
  exit 1
fi
exec python3 "$workspace_dir/log_run.py" isaac -- \
  ros2 launch ur10_isaac_moveit_config isaac.launch.py "$@"
