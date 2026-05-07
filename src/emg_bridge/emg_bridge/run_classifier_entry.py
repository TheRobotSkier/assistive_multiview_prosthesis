#!/usr/bin/env python3
"""Thin entry point for the EMG classifier runner.

This wrapper exists so that `ros2 run emg_bridge run_classifier` works.
It simply delegates to scripts/run_classifier.py.
"""

import runpy
import sys
import os

# Add scripts dir to path so run_classifier can be found
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))


def main():
    # Execute the run_classifier script
    runpy.run_module("run_classifier", run_name="__main__")


if __name__ == "__main__":
    main()
