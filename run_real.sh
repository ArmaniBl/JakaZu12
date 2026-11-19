#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
robot_ip=${1:-10.5.5.100}
servo_lpf_cutoff_hz=${2:-0.5}
if [[ $# -ge 2 ]]; then shift 2; elif [[ $# -ge 1 ]]; then shift; fi

if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
  echo "Workspace is not built. Run ./build.sh --real first." >&2
  exit 1
fi

set +u
source "$workspace_dir/setup_isaac.bash"
set -u
export LD_LIBRARY_PATH="$workspace_dir/src/jaka_planner/lib:${LD_LIBRARY_PATH:-}"
if ! ros2 pkg prefix jaka_planner >/dev/null 2>&1; then
  echo 'Build the physical JAKA packages first: ./build.sh --real' >&2
  exit 1
fi

exec python3 "$workspace_dir/log_run.py" real -- \
  ros2 launch jaka_zu12_moveit_config real_robot.launch.py \
  ip:="$robot_ip" \
  auto_enable:=false \
  scan_preview:=true \
  scan_preview_script:="$workspace_dir/run_scan_preview.sh" \
  servo_lpf_cutoff_hz:="$servo_lpf_cutoff_hz" "$@"
