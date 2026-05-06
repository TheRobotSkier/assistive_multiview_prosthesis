"""Launch a single D435 camera through the official realsense2 package launch file.

USB 2.0 note: The D435 connected over USB 2.0/2.1 cannot stream depth + color
simultaneously. Color is enabled by default because pointcloud.enable requires the
color stream. Set enable_color:=false if on USB 2.0 (pointcloud won't work).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription


DEFAULT_SERIAL = os.getenv('REALSENSE_SERIAL_NO', '')
_DEFAULT_DEPTH_PROFILE = os.getenv('REALSENSE_DEPTH_PROFILE', '640x480x15')
_DEFAULT_COLOR_PROFILE = os.getenv('REALSENSE_COLOR_PROFILE', '640x480x15')


def _camera_launch(context, *_args, **_kwargs):
    launch_arguments = {
        'camera_namespace': 'cam1',
        'camera_name': 'd435_1',
        'enable_color': LaunchConfiguration('enable_color').perform(context),
        'pointcloud.enable': 'true',
        'align_depth.enable': 'true',
        'enable_infra1': 'false',
        'enable_infra2': 'false',
        'initial_reset': LaunchConfiguration('initial_reset').perform(context),
        'depth_module.depth_profile': LaunchConfiguration('depth_profile').perform(context),
        'rgb_camera.color_profile': LaunchConfiguration('color_profile').perform(context),
    }
    serial_no = LaunchConfiguration('serial_no').perform(context).strip()
    if serial_no:
        launch_arguments['serial_no'] = serial_no

    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py',
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rs_launch),
            launch_arguments=launch_arguments.items(),
        ),
        TimerAction(
            period=6.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'param', 'set', '/cam1/d435_1', 'pointcloud.enable', 'true']
                )
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_no',
            default_value=DEFAULT_SERIAL,
            description='Serial number of the D435 camera to launch. Empty selects the first available camera.',
        ),
        DeclareLaunchArgument(
            'enable_color',
            default_value='true',
            description='Enable RGB color stream (required for pointcloud.enable). Set false on USB 2.0 (disables pointcloud).',
        ),
        DeclareLaunchArgument(
            'initial_reset',
            default_value='false',
            description='Reset the camera on startup. Disabled by default to avoid device renumbering races.',
        ),
        DeclareLaunchArgument(
            'depth_profile',
            default_value=_DEFAULT_DEPTH_PROFILE,
            description='Depth stream profile in WIDTHxHEIGHTxFPS format.',
        ),
        DeclareLaunchArgument(
            'color_profile',
            default_value=_DEFAULT_COLOR_PROFILE,
            description='Color stream profile in WIDTHxHEIGHTxFPS format.',
        ),
        OpaqueFunction(function=_camera_launch),
    ])

