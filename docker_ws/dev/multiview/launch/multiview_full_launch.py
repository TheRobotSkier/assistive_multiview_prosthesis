"""Launch cameras + static TF + pointcloud fusion node."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

_HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    cameras = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(_HERE, 'two_d435_launch.py')))

    # Static TF: cam2 optical frame relative to cam1 optical frame.
    # Replace translation+rotation with values from CharUco calibration
    # (multiview/intrinsics/). Identity used until calibrated.
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '0', '0', '0',          # x y z (meters) — update after calibration
            '0', '0', '0', '1',     # qx qy qz qw — identity
            'd435_1_color_optical_frame',
            'd435_2_color_optical_frame',
        ],
        name='cam2_to_cam1_tf',
        output='screen',
    )

    # Start fusion node after cameras and TF are ready
    fusion = TimerAction(period=8.0, actions=[
        Node(
            executable='python3',
            arguments=['/ros_ws/nodes/pointcloud_fusion_node.py'],
            name='pointcloud_fusion',
            output='screen',
        )
    ])

    return LaunchDescription([cameras, static_tf, fusion])
