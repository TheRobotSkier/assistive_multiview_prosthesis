"""Launch two RealSense D435 cameras with pointcloud fusion.

Launches two realsense2_camera_node instances (one per serial), a static TF
publisher relating their depth optical frames, and the pointcloud_fusion_node
that merges both clouds into /fused_pointcloud.

Usage (inside container):
    ros2 launch /ros_ws/launch/two_d435_launch.py

Configurable via environment variables:
    CAM1_SERIAL       Serial for camera 1 (default: 829212072207)
    CAM2_SERIAL       Serial for camera 2 (default: 827112072033)
    CAM2_OFFSET_X     X-offset from cam1 to cam2 depth frame (default: 0.15)

Design note:
    Uses ExecuteProcess (not Node) for the realsense nodes because the launch
    framework's YAML parameter serialisation writes all-numeric serial numbers
    without quotes, causing the realsense node to reject them as "invalid type:
    parameter 'serial_no' is of type {string}, setting it to {integer}".
    The workaround: pass serial_no via --ros-args -p with explicit YAML single
    quotes (-p "serial_no:='830213023028'") which forces string interpretation.
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
import launch_ros.actions


_REALSENSE_NODE = '/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node'
_DEPTH_PROFILE = '640x480x6'
_COLOR_PROFILE = '640x480x6'

_CAM1_SERIAL = os.environ.get('CAM1_SERIAL', '829212072207')
_CAM2_SERIAL = os.environ.get('CAM2_SERIAL', '827112072033')
_CAM2_OFFSET_X = os.environ.get('CAM2_OFFSET_X', '0.5')


def _realsense_cmd(serial: str, namespace: str, node_name: str) -> list:
    """Build ExecuteProcess cmd for one realsense2_camera_node."""
    return [
        _REALSENSE_NODE,
        '--ros-args', '--log-level', 'info',
        '-r', f'__node:={node_name}',
        '-r', f'__ns:=/{namespace}',
        # YAML single quotes force string type (see design note above)
        '-p', f"serial_no:='{serial}'",
        '-p', 'enable_color:=true',
        '-p', f'depth_module.depth_profile:={_DEPTH_PROFILE}',
        '-p', f'rgb_camera.color_profile:={_COLOR_PROFILE}',
        '-p', 'pointcloud.enable:=true',
        '-p', 'align_depth.enable:=true',
        '-p', 'enable_infra1:=false',
        '-p', 'enable_infra2:=false',
        '-p', 'initial_reset:=false',
    ]


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=f'Starting dual D435 cameras (cam1={_CAM1_SERIAL}, cam2={_CAM2_SERIAL})'),

        # --- Camera 1 ---
        ExecuteProcess(
            cmd=_realsense_cmd(_CAM1_SERIAL, 'cam1', 'd435_1'),
            output='screen',
            emulate_tty=True,
        ),

        # --- Camera 2 ---
        ExecuteProcess(
            cmd=_realsense_cmd(_CAM2_SERIAL, 'cam2', 'd435_2'),
            output='screen',
            emulate_tty=True,
        ),

        # --- Static TF: cam2_depth frame relative to cam1_depth frame ---
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='cam2_to_cam1_tf',
            arguments=[
                _CAM2_OFFSET_X, '0.0', '0.0',   # translation
                '0.0', '0.0', '0.0', '1.0',     # rotation (identity)
                'd435_1_depth_optical_frame',    # parent
                'd435_2_depth_optical_frame',    # child
            ],
            output='screen',
            emulate_tty=True,
        ),

        # Pointcloud fusion removed per user feedback: individual clouds only
    ])
