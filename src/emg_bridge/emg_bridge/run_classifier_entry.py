#!/usr/bin/env python3
"""Thin entry point for the EMG classifier runner.

This wrapper exists so that `ros2 run emg_bridge run_classifier` works.
It extracts --model-dir from ROS parameters (or regular arguments) and
delegates to scripts/run_classifier.py.
"""

import runpy
import sys
import os

# Add scripts dir to path so run_classifier can be found
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


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
    skip_next = False
    while i < len(args):
        if skip_next:
            skip_next = False
            i += 1
            continue
        if args[i] == "--ros-args":
            # Scan until -- or end for -p name:=value
            i += 1
            ros_section = []
            while i < len(args) and args[i] != "--":
                ros_section.append(args[i])
                i += 1
            # Skip the terminating -- if present
            if i < len(args) and args[i] == "--":
                i += 1
            # Parse -p entries
            j = 0
            while j < len(ros_section):
                if ros_section[j] == "-p" and j + 1 < len(ros_section):
                    param = ros_section[j + 1]
                    if param.startswith(f"{name}:="):
                        value = param[len(name) + 2 :]
                    else:
                        cleaned.extend(["-p", param])
                    j += 2
                else:
                    cleaned.append(ros_section[j])
                    j += 1
            continue
        cleaned.append(args[i])
        i += 1
    return value, cleaned


def main():
    # Check for model_dir in ROS parameters or regular args
    model_dir, remaining = _extract_ros_param(sys.argv[1:], "model_dir")

    # Also check regular --model-dir in remaining args
    i = 0
    while i < len(remaining):
        if remaining[i] == "--model-dir" and i + 1 < len(remaining):
            model_dir = remaining[i + 1]
            break
        i += 1

    # If found via ROS param but not in regular args, inject it
    if model_dir is not None and "--model-dir" not in remaining:
        remaining = ["--model-dir", model_dir] + remaining

    # Replace sys.argv so the script sees the intended arguments
    sys.argv = [sys.argv[0]] + remaining

    # Execute the run_classifier script
    runpy.run_module("run_classifier", run_name="__main__")


if __name__ == "__main__":
    main()
