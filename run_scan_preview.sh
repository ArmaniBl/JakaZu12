#!/usr/bin/env bash
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$workspace_dir/setup_isaac.bash"
# Keep one preview publisher per user and ROS domain, including manual launches.
exec 9>"/tmp/jaka-scan-preview-${UID}-${ROS_DOMAIN_ID:-0}.lock"
if ! flock -n 9; then
  echo 'Scanner preview is already running in this ROS domain.'
  exit 0
fi
exec "$workspace_dir/.venv-rangevision/bin/python" "$workspace_dir/src/ur10_isaac_moveit_config/scripts/scan_preview.py" --continuous --auto-fit "$@"
