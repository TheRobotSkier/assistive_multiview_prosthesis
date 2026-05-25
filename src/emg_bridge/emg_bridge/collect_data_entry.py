#!/usr/bin/env python3
"""Thin entry point for the EMG data collection script.

This wrapper exists so that `ros2 run emg_bridge collect_data` works.
It simply delegates to scripts/collect_data.py.
"""

import runpy
import sys


def main():
    runpy.run_module("scripts.collect_data", run_name="__main__")


if __name__ == "__main__":
    main()
