"""Launch a single D435 camera by directly executing realsense2_camera_node.

USB 2.0 note: The D435 connected over USB 2.0/2.1 cannot stream depth + color
simultaneously. Color is enabled by default because pointcloud.enable requires the
color stream. Set REALSENSE_ENABLE_COLOR=false if on USB 2.0 (pointcloud won't work).

Configurable via environment variables:
    CAM1_SERIAL              Serial number (default: 829212072207)
    REALSENSE_ENABLE_COLOR   "true" or "false" (default: true)
    REALSENSE_INITIAL_RESET  "true" or "false" (default: false)

Design note:
    Uses ExecuteProcess (not Node/IncludeLaunchDescription) to avoid a YAML
    serialisation bug: serial numbers composed of digits are written unquoted into
    the temporary params file, parsed as integers, and rejected by the realsense
    node with "parameter 'serial_no' has invalid type". The workaround is to pass
    serial_no via --ros-args -p with YAML single-quotes surrounding the value:
      -p "serial_no:='830213023028'"
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo

_REALSENSE_NODE = '/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node'
_DEPTH_PROFILE = '640x480x15'

_CAM1_SERIAL = os.environ.get('CAM1_SERIAL', os.environ.get('REALSENSE_SERIAL_NO', '829212072207'))
_ENABLE_COLOR = os.environ.get('REALSENSE_ENABLE_COLOR', 'true')
_INITIAL_RESET = os.environ.get('REALSENSE_INITIAL_RESET', 'false')


def _realsense_cmd() -> list:
    """Build the ExecuteProcess cmd list for one realsense2_camera_node."""
    cmd = [
        _REALSENSE_NODE,
        '--ros-args', '--log-level', 'info',
        '-r', '__node:=d435_1',
        '-r', '__ns:=/cam1',
        '-p', f'depth_module.depth_profile:={_DEPTH_PROFILE}',
        '-p', 'pointcloud.enable:=true',
        '-p', 'align_depth.enable:=true',
        '-p', f'initial_reset:={_INITIAL_RESET}',
        '-p', f'enable_color:={_ENABLE_COLOR}',
    ]
    # Serial number: YAML single-quotes force string type
    if _CAM1_SERIAL:
        cmd.extend(['-p', f"serial_no:='{_CAM1_SERIAL}'"])
    return cmd


def generate_launch_description():
    serial_display = _CAM1_SERIAL if _CAM1_SERIAL else '(auto-detect)'
    return LaunchDescription([
        LogInfo(msg=f'Starting single D435 camera (serial={serial_display})'),
        ExecuteProcess(
            cmd=_realsense_cmd(),
            output='screen',
            emulate_tty=True,
        ),
    ])
