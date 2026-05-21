#!/usr/bin/env python3
"""Thin entry point for the EMG training script.

This wrapper exists so that `ros2 run emg_bridge train` works.
It simply delegates to scripts/train.py.
"""

import runpy
import sys
import os

# Add scripts dir to path so train can be found
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


def main():
    # Execute the train script
    runpy.run_module("train", run_name="__main__")


if __name__ == "__main__":
    main()
