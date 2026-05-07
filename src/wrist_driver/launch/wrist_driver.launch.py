"""Wrist driver launch file.

Launches the Dynamixel wrist motor driver with parameters from config.
Supports a mock mode that publishes fake data without hardware.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_mock_arg = DeclareLaunchArgument(
        'use_mock',
        default_value='false',
        description='Publish fake wrist data instead of connecting to hardware',
    )

    params_file = PathJoinSubstitution([
        FindPackageShare('wrist_driver'),
        'config',
        'wrist_params.yaml',
    ])

    wrist_driver_node = Node(
        package='wrist_driver',
        executable='wrist_driver_node',
        name='wrist_driver',
        parameters=[params_file],
        condition=UnlessCondition(LaunchConfiguration('use_mock')),
        output='screen',
    )

    return LaunchDescription([
        use_mock_arg,
        wrist_driver_node,
    ])
