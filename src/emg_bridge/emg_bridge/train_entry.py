#!/usr/bin/env python3
"""Entry point for `ros2 run emg_bridge train`.

Delegates to emg_bridge.scripts.train (no path resolution needed).
"""

from emg_bridge.scripts.train import main


if __name__ == "__main__":
    main()
