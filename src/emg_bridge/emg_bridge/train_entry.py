#!/usr/bin/env python3
"""Thin entry point for the EMG training script.

This wrapper exists so that `ros2 run emg_bridge train` works.
It simply delegates to scripts/train.py.
"""

import runpy
import sys


def main():
    runpy.run_module("scripts.train", run_name="__main__")


if __name__ == "__main__":
    main()
