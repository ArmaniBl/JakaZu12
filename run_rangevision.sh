#!/usr/bin/env bash
set -eo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$workspace_dir/setup_isaac.bash"
if [[ ! -x "$workspace_dir/.venv-rangevision/bin/python" ]]; then
  echo 'Run ./setup_rangevision.sh first.' >&2
  exit 1
fi
package_prefix=$(ros2 pkg prefix ur10_isaac_moveit_config)
exec "$workspace_dir/.venv-rangevision/bin/python" "$package_prefix/lib/ur10_isaac_moveit_config/rangevision.py" "$@"
