#!/usr/bin/env python3
"""Capture a ROS run without adding traffic to the robot connection."""

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=["real", "simulation", "recovery"])
    parser.add_argument("command", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == "--":
        command = command[1:]
    if not command:
        parser.error("a command is required")

    workspace = Path(__file__).resolve().parent
    root = workspace / "diagnostics"
    root.mkdir(parents=True, exist_ok=True)
    session = Path(tempfile.mkdtemp(
        prefix=datetime.now().strftime("%Y-%m-%d_%H-%M-%S_") + args.mode + "_",
        dir=root,
    ))
    ros_logs = session / "ros"
    ros_logs.mkdir()
    metadata = {
        "mode": args.mode,
        "started_utc": datetime.now(timezone.utc).isoformat(),
        "command": command,
        "workspace": str(workspace),
        "environment": {key: os.environ.get(key) for key in (
            "ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION",
            "ROS_LOCALHOST_ONLY", "DISPLAY", "WAYLAND_DISPLAY",
        )},
        "sha256": {},
    }
    for relative in (
        "src/jaka_zu12_moveit_config/config",
        "src/jaka_zu12_moveit_config/launch/real_robot.launch.py",
        "src/jaka_zu12_moveit_config/launch/simulation.launch.py",
        "src/jaka_zu12_moveit_config/scripts/add_environment.py",
        "src/jaka_description/urdf/jaka_zu12.urdf",
        "src/jaka_planner/src/moveit_server.cpp",
        "src/jaka_planner/lib/libjakaAPI.so",
    ):
        source = workspace / relative
        files = sorted(source.rglob("*")) if source.is_dir() else [source]
        for path in files:
            if not path.is_file() or "__pycache__" in path.parts:
                continue
            name = str(path.relative_to(workspace))
            metadata["sha256"][name] = hashlib.sha256(path.read_bytes()).hexdigest()
            if path.suffix != ".so":
                destination = session / "snapshot" / name
                destination.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(path, destination)

    def save_metadata():
        (session / "session.json").write_text(
            json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
        )

    save_metadata()
    print(f"Diagnostics: {session}", flush=True)
    environment = os.environ.copy()
    environment["ROS_LOG_DIR"] = str(ros_logs)
    environment["RCUTILS_COLORIZED_OUTPUT"] = "0"
    environment["PYTHONUNBUFFERED"] = "1"
    process = None

    def forward_signal(signum, _frame):
        metadata["last_signal"] = signum
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signum)
            except ProcessLookupError:
                pass

    signal.signal(signal.SIGINT, forward_signal)
    signal.signal(signal.SIGTERM, forward_signal)
    exit_code = 1
    try:
        with (session / "console.log").open("w", encoding="utf-8", buffering=1) as log:
            process = subprocess.Popen(
                command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                text=True, errors="replace", env=environment,
                start_new_session=True,
            )
            for line in process.stdout:
                log.write(f"{datetime.now(timezone.utc).isoformat()} {line}")
                try:
                    sys.stdout.write(line)
                    sys.stdout.flush()
                except BrokenPipeError:
                    pass
            exit_code = process.wait()
    except OSError as error:
        metadata["logging_error"] = str(error)
        print(f"Run failed: {error}", file=sys.stderr)
    finally:
        if process is not None and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            except ProcessLookupError:
                process.wait()
        metadata["finished_utc"] = datetime.now(timezone.utc).isoformat()
        metadata["exit_code"] = exit_code
        save_metadata()
    return exit_code if exit_code >= 0 else 128 - exit_code


if __name__ == "__main__":
    sys.exit(main())
