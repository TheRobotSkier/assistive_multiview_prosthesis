"""Launch one or two D435 cameras via the official realsense2 package launch file."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration


_DEFAULT_CAM1_SERIAL = os.getenv('REALSENSE_CAM1_SERIAL', os.getenv('REALSENSE_SERIAL_NO', ''))
_DEFAULT_CAM2_SERIAL = os.getenv('REALSENSE_CAM2_SERIAL', '')
_DEFAULT_DEPTH_PROFILE = os.getenv('REALSENSE_DEPTH_PROFILE', '640x480x15')
_DEFAULT_COLOR_PROFILE = os.getenv('REALSENSE_COLOR_PROFILE', '640x480x15')


def _camera_actions(context, *_args, **_kwargs):
    rs_launch = os.path.join(
        get_package_share_directory('realsense2_camera'),
        'launch',
        'rs_launch.py',
    )
    depth_profile = LaunchConfiguration('depth_profile').perform(context)
    color_profile = LaunchConfiguration('color_profile').perform(context)
    enable_color = LaunchConfiguration('enable_color').perform(context)
    initial_reset = LaunchConfiguration('initial_reset').perform(context)

    actions = []
    for namespace, camera_name, serial_config in (
        ('cam1', 'd435_1', LaunchConfiguration('cam1_serial').perform(context).strip()),
        ('cam2', 'd435_2', LaunchConfiguration('cam2_serial').perform(context).strip()),
    ):
        if namespace == 'cam2' and not serial_config:
            continue

        launch_arguments = {
            'camera_namespace': namespace,
            'camera_name': camera_name,
            'enable_color': enable_color,
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'initial_reset': initial_reset,
            'depth_module.depth_profile': depth_profile,
            'rgb_camera.color_profile': color_profile,
        }
        if serial_config:
            launch_arguments['serial_no'] = serial_config

        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch),
                launch_arguments=launch_arguments.items(),
            )
        )
        actions.append(
            TimerAction(
                period=6.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'param', 'set', f'/{namespace}/{camera_name}', 'pointcloud.enable', 'true']
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'cam1_serial',
            default_value=_DEFAULT_CAM1_SERIAL,
            description='Serial number for the first D435 camera. Empty selects the first available camera.',
        ),
        DeclareLaunchArgument(
            'cam2_serial',
            default_value=_DEFAULT_CAM2_SERIAL,
            description='Serial number for the second D435 camera. Leave empty to run in single-camera mode.',
        ),
        DeclareLaunchArgument(
            'enable_color',
            default_value='false',
            description='Enable RGB color stream (requires USB 3.0+).',
        ),
        DeclareLaunchArgument(
            'initial_reset',
            default_value='false',
            description='Reset cameras on startup. Disabled by default to avoid device renumbering races.',
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
        OpaqueFunction(function=_camera_actions),
    ])

