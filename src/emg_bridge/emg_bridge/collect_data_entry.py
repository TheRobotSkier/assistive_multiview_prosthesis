#!/usr/bin/env python3
"""Thin entry point for the EMG data collection script.

This wrapper exists so that `ros2 run emg_bridge collect_data` works.
It simply delegates to scripts/collect_data.py.
"""

import runpy
import sys
import os

# Add scripts dir to path so collect_data can be found
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


def main():
    # Execute the collect_data script
    runpy.run_module("collect_data", run_name="__main__")


if __name__ == "__main__":
    main()
