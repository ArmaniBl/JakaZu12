#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

set +u
source /opt/ros/humble/setup.bash
set -u

cd "$workspace_dir"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF

echo
echo "Build complete. In each new terminal run:"
echo "  source $workspace_dir/install/setup.bash"
