#!/usr/bin/env python3
"""Entry point for `ros2 run emg_bridge collect_data`.

Delegates to emg_bridge.scripts.collect_data (no path resolution needed).
"""

from emg_bridge.scripts.collect_data import main


if __name__ == "__main__":
    main()
