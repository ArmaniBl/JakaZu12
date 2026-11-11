#!/usr/bin/env bash
set -euo pipefail

workspace_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
robot_ip=${1:-10.5.5.100}

if [[ ! $robot_ip =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
  echo "Usage: $0 [robot-ip]" >&2
  exit 2
fi

echo "Local WSL addresses:"
ip -4 -brief address
echo

windows_ips=""
if command -v powershell.exe >/dev/null 2>&1; then
  windows_ips=$(powershell.exe -NoProfile -Command \
    "Get-NetIPAddress -AddressFamily IPv4 | Select-Object -ExpandProperty IPAddress" \
    2>/dev/null | tr -d '\r' || true)
  echo "Local Windows addresses:"
  printf '%s\n\n' "$windows_ips"
fi

if ip -4 -o address show | grep -qE "[[:space:]]${robot_ip}/" || \
   grep -Fxq "$robot_ip" <<<"$windows_ips"; then
  echo "ERROR: ${robot_ip} is assigned to this computer, not to the robot." >&2
  exit 2
fi

echo "Route to ${robot_ip}:"
ip route get "$robot_ip"
echo

ping_ok=no
if ping -c 2 -W 2 "$robot_ip"; then
  ping_ok=yes
fi

sdk_ok=no
if timeout 3 bash -c "</dev/tcp/${robot_ip}/10004" 2>/dev/null; then
  sdk_ok=yes
  echo "JAKA SDK TCP 10004: reachable"
else
  echo "JAKA SDK TCP 10004: not reachable"
fi

set +u
source "$workspace_dir/setup_isaac.bash"
set -u

for package in jaka_planner jaka_zu12_moveit_config; do
  ros2 pkg prefix "$package" >/dev/null || {
    echo "Missing package ${package}. Run ./build.sh --real first." >&2
    exit 1
  }
done

if [[ $ping_ok == yes || $sdk_ok == yes ]]; then
  echo "Controller reachable. The ZU12 workspace is ready."
else
  echo "Controller not reached. Check Wi-Fi and the robot IP." >&2
  exit 1
fi
