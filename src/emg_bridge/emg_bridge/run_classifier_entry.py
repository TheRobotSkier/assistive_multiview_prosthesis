#!/usr/bin/env python3
"""Entry point for `ros2 run emg_bridge run_classifier`.

Extracts --model-dir from ROS parameters (or regular arguments) and
delegates to emg_bridge.scripts.run_classifier (no path resolution needed).
"""

import sys

from emg_bridge.scripts.run_classifier import main as classifier_main


def _extract_ros_param(args: list[str], name: str) -> tuple[str | None, list[str]]:
    """Extract a ROS 2 parameter value from sys.argv-style arguments.

    Supports:
        --ros-args -p name:=value --
        --ros-args -p name:=value (implicit end)
    Returns (value, remaining_args).
    """
    i = 0
    value = None
    cleaned = []
    while i < len(args):
        if args[i] == "--ros-args":
            i += 1
            ros_section = []
            while i < len(args) and args[i] != "--":
                ros_section.append(args[i])
                i += 1
            if i < len(args) and args[i] == "--":
                i += 1
            j = 0
            while j < len(ros_section):
                if ros_section[j] == "-p" and j + 1 < len(ros_section):
                    param = ros_section[j + 1]
                    if param.startswith(f"{name}:="):
                        value = param[len(name) + 2 :]
                    else:
                        cleaned.extend(["-p", param])
                    j += 2
                elif ros_section[j] == "-r" and j + 1 < len(ros_section):
                    # Strip ROS remapping arguments (e.g. -r __node:=name)
                    j += 2
                elif ros_section[j] == "--params-file" and j + 1 < len(ros_section):
                    # Strip params-file arguments
                    j += 2
                elif ros_section[j].startswith("-"):
                    # Strip other ROS flags (e.g. --log-level)
                    j += 1
                else:
                    j += 1
            continue
        cleaned.append(args[i])
        i += 1
    return value, cleaned


def main():
    # Check for model_dir in ROS parameters or regular args
    model_dir, remaining = _extract_ros_param(sys.argv[1:], "model_dir")

    i = 0
    while i < len(remaining):
        if remaining[i] == "--model-dir" and i + 1 < len(remaining):
            model_dir = remaining[i + 1]
            break
        i += 1

    if model_dir is not None and "--model-dir" not in remaining:
        remaining = ["--model-dir", model_dir] + remaining

    sys.argv = [sys.argv[0]] + remaining
    classifier_main()


if __name__ == "__main__":
    main()
