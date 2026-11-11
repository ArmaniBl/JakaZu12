#!/usr/bin/env bash
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$workspace_dir/setup_isaac.bash"
exec ros2 run ur10_isaac_moveit_config welding_web.py --workspace "$workspace_dir" "$@"
