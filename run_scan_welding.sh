#!/usr/bin/env bash
set -euo pipefail
workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
exec "$workspace_dir/run_web.sh" --robot jaka_real --port 8090 "$@"
