#!/usr/bin/env bash
set -euo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
python3 -m venv --system-site-packages "$workspace_dir/.venv-rangevision"
"$workspace_dir/.venv-rangevision/bin/python" -m pip install 'scipy==1.15.3'
mkdir -p "$workspace_dir/scans"
echo 'RangeVision processing environment ready. Next: ./build.sh --real'
