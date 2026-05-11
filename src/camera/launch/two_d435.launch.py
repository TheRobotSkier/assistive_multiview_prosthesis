"""Launch two RealSense D435 cameras with individual pointcloud streams.

Launches two realsense2_camera_node instances (one per serial) and a static TF
publisher relating their depth optical frames for alignment in RViz.

Configurable via environment variables:
    CAM1_SERIAL       Serial for camera 1 (default: 829212072207)
    CAM2_SERIAL       Serial for camera 2 (default: 827112072033)
    CAM2_OFFSET_X     X-offset from cam1 to cam2 depth frame (default: 0.5)

Design note:
    Uses ExecuteProcess (not Node) for the realsense nodes because the launch
    framework's YAML parameter serialisation writes all-numeric serial numbers
    without quotes, causing the realsense node to reject them as "invalid type:
    parameter 'serial_no' is of type {string}, setting it to {integer}".
    The workaround: pass serial_no via --ros-args -p with explicit YAML single
    quotes (-p "serial_no:='SERIAL'") which forces string interpretation.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
import launch_ros.actions


_REALSENSE_NODE = '/opt/ros/jazzy/lib/realsense2_camera/realsense2_camera_node'
_DEPTH_PROFILE = '640x480x6'
_COLOR_PROFILE = '640x480x6'

_CAM1_SERIAL = os.environ.get('CAM1_SERIAL', '829212072207')
_CAM2_SERIAL = os.environ.get('CAM2_SERIAL', '827112072033')
_CAM2_OFFSET_X = os.environ.get('CAM2_OFFSET_X', '0.5')


def _realsense_cmd(serial: str, namespace: str, node_name: str, tf_prefix: str) -> list:
    """Build ExecuteProcess cmd for one realsense2_camera_node."""
    return [
        _REALSENSE_NODE,
        '--ros-args', '--log-level', 'info',
        '-r', f'__node:={node_name}',
        '-r', f'__ns:=/{namespace}',
        # YAML single quotes force string type (see design note above)
        '-p', f"serial_no:='{serial}'",
        '-p', f"tf_prefix:='{tf_prefix}'",
        '-p', "camera_name:='camera'",
        '-p', 'enable_color:=true',
        '-p', f'depth_module.depth_profile:={_DEPTH_PROFILE}',
        '-p', f'rgb_camera.color_profile:={_COLOR_PROFILE}',
        '-p', 'pointcloud.enable:=true',
        '-p', 'pointcloud.stream_filter:=2',
        '-p', 'align_depth.enable:=true',
        '-p', 'enable_infra1:=false',
        '-p', 'enable_infra2:=false',
        '-p', 'initial_reset:=false',
    ]


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=f'Starting dual D435 cameras (cam1={_CAM1_SERIAL}, cam2={_CAM2_SERIAL})'),

        # --- Camera 1 (base, unmoving) ---
        ExecuteProcess(
            cmd=_realsense_cmd(_CAM1_SERIAL, 'cam1', 'd435_1', 'cam1'),
            output='screen',
            emulate_tty=True,
        ),

        # --- Camera 2 (mounted on the hand) ---
        ExecuteProcess(
            cmd=_realsense_cmd(_CAM2_SERIAL, 'cam2', 'd435_2', 'cam2'),
            output='screen',
            emulate_tty=True,
        ),

        # Pointcloud fusion removed per user feedback: individual clouds only
    ])
