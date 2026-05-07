"""Launch the full multiview pipeline: dual cameras with static TF alignment.

Simply delegates to two_d435_launch.py (which handles the serial_no YAML quoting
workaround and publishes individual pointcloud streams + static TF).

Configurable via environment variables (passed through to two_d435_launch.py):
    CAM1_SERIAL       Serial for camera 1 (default: 829212072207)
    CAM2_SERIAL       Serial for camera 2 (default: 827112072033)
    CAM2_OFFSET_X     X-offset from cam1 to cam2 depth frame (default: 0.5)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource


_LAUNCH_FILE = os.path.join(
    os.path.dirname(__file__),
    'two_d435_launch.py',
)


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=f'Starting multiview full pipeline'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(_LAUNCH_FILE),
        ),
    ])
