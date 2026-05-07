"""Launch both D435 cameras with pointcloud enabled.

Fixes applied:
- Serial numbers read from env vars (CAM1_SERIAL, CAM2_SERIAL)
- Use ExecuteProcess to avoid YAML integer serial issue
- Disable infra1/infra2 streams (USB bandwidth)
- Reduce FPS to 6 (USB bandwidth)
- Enable pointcloud at launch with RGB color texture
- Remove broken TimerAction workaround
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def get_serial(name: str, default: str) -> str:
    """Read camera serial from env var, falling back to default.

    The realsense2_camera_node expects serial_no as a YAML string.
    All-numeric serials are misinterpreted as integers when the ROS
    launch framework generates a temporary YAML parameter file, so we
    use ExecuteProcess with explicit YAML single-quoting around the value.
    """
    return os.environ.get(name, default)


CAM1_SERIAL = get_serial('CAM1_SERIAL', '829212072207')
CAM2_SERIAL = get_serial('CAM2_SERIAL', '827112072033')


def _camera_cmd(namespace: str, camera_name: str, serial: str):
    """Build ExecuteProcess arguments for one realsense2_camera_node."""
    return [
        'ros2', 'run', 'realsense2_camera', 'realsense2_camera_node',
        '--ros-args',
        '-r', f'__ns:={namespace}',
        '-p', f"serial_no:='{serial}'",
        '-p', f"camera_name:='{camera_name}'",
        # Stream configuration
        '-p', 'enable_sync:=True',
        '-p', 'align_depth.enable:=True',
        '-p', 'enable_color:=True',
        # Disable infra to reduce USB bandwidth
        '-p', 'enable_infra1:=False',
        '-p', 'enable_infra2:=False',
        # Reduce FPS for dual-camera USB bandwidth sharing
        '-p', "depth_module.depth_profile:='640x480x6'",
        '-p', "rgb_camera.color_profile:='640x480x6'",
        # Enable pointcloud with RGB texture at launch
        '-p', 'pointcloud.enable:=True',
        '-p', 'pointcloud.stream_filter:=2',
        '-p', 'pointcloud.allow_no_texture_points:=False',
    ]


def generate_launch_description():
    cam1 = ExecuteProcess(
        cmd=_camera_cmd('/cam1/d435_1', 'd435_1', CAM1_SERIAL),
        output='screen',
    )
    cam2 = ExecuteProcess(
        cmd=_camera_cmd('/cam2/d435_2', 'd435_2', CAM2_SERIAL),
        output='screen',
    )

    return LaunchDescription([cam1, cam2])
