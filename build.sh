#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

set +u
source "$workspace_dir/setup_isaac.bash"
set -u

cd "$workspace_dir"
packages=(ur10_isaac_moveit_config jaka_description jaka_planner jaka_zu12_moveit_config)
if [[ $# != 0 && ( $# != 1 || $1 != --real ) ]]; then
  echo 'Usage: ./build.sh [--real]' >&2
  exit 2
fi
colcon build --base-paths src --symlink-install --packages-select "${packages[@]}" --cmake-args -DBUILD_TESTING=OFF

echo
echo "Build complete. In each new terminal run:"
echo "  source $workspace_dir/install/setup.bash"
