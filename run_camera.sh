#!/usr/bin/env bash
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$workspace_dir/setup_isaac.bash"
exec rviz2 -d "$workspace_dir/isaac/camera.rviz" "$@"
