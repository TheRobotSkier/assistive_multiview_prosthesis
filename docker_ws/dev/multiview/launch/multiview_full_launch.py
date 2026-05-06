"""Launch the full multiview pipeline: dual cameras, static TF, pointcloud fusion.

This delegates the actual camera launch to two_d435_launch.py (which handles the
serial_no YAML quoting workaround) and adds the pointcloud_fusion_node.

Configurable via environment variables (passed through to two_d435_launch.py):
    CAM1_SERIAL       Serial for camera 1 (default: 829212072207)
    CAM2_SERIAL       Serial for camera 2 (default: 827112072033)
    CAM2_OFFSET_X     X-offset from cam1 to cam2 depth frame (default: 0.15)
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


LAUNCH_FILE = os.path.join(
    os.path.dirname(__file__),
    'two_d435_launch.py',
)


def generate_launch_description():
    # Pass through environment variables needed by two_d435_launch.py.
    # The launched file reads os.environ directly, so no explicit
    # launch_arguments are needed.
    return LaunchDescription([
        LogInfo(msg=f'Starting multiview full pipeline'),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(LAUNCH_FILE),
        ),
    ])
