"""Digital Twin launch file.

Orchestrates the full digital twin pipeline:
  1. Mia Hand MuJoCo simulation (InteractiveSystemInterface) [optional, not yet ported]
  2. Pointcloud relay (/fused_pointcloud -> /segmentation/input_cloud)
  3. Segmentation ROS2 node
  4. Click relay (RViz /clicked_point -> segmentation clicks)
  5. Grasp preshaping service bridge
  6. Grasp proximity controller
  7. RViz (delayed start)
  8. Optional hand trajectory test node

Prerequisites (must be running first):
  - multiview_full       : publishes /fused_pointcloud from real D435s
  - segmentation_inference: HTTP inference server on :5678
  - rust_build           : one-shot, ensures libgrasp_preshaping.so exists

Usage:
  ros2 launch prosthesis_launch digital_twin.launch.py
  ros2 launch prosthesis_launch digital_twin.launch.py use_trajectory:=true
  ros2 launch prosthesis_launch digital_twin.launch.py inference_url:=http://192.168.1.100:5678
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # -- Launch arguments --------------------------------------------------
    use_trajectory_arg = DeclareLaunchArgument(
        'use_trajectory',
        default_value='false',
        description='Include automated hand approach test node',
    )
    inference_url_arg = DeclareLaunchArgument(
        'inference_url',
        default_value='http://127.0.0.1:5678',
        description='Segmentation inference server URL',
    )
    segmentation_cubeedge_arg = DeclareLaunchArgument(
        'segmentation_cubeedge',
        default_value='0.05',
        description='Click cube half-width (m)',
    )
    publish_initial_commands_arg = DeclareLaunchArgument(
        'publish_initial_commands',
        default_value='false',
        description='If true, preshaping bridge sends immediate joint commands on service call. '
                    'Digital twin sets this to false so the proximity controller owns all commands.',
    )

    use_trajectory = LaunchConfiguration('use_trajectory')
    inference_url = LaunchConfiguration('inference_url')
    segmentation_cubeedge = LaunchConfiguration('segmentation_cubeedge')
    publish_initial_commands = LaunchConfiguration('publish_initial_commands')

    # -- 1. Mia Hand MuJoCo simulation -------------------------------------
    # NOTE: The mia_hand_mujoco package with the InteractiveSystemInterface
    # has not been ported to the new src/ structure yet (14k+ lines of C++
    # MuJoCo code). Once ported, uncomment the block below.
    #
    # mia_hand_sim_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('mia_hand_mujoco'), 'launch',
    #             'mia_hand_system_interface_launch.py'
    #         ])
    #     ]),
    #     launch_arguments={
    #         'hardware_plugin': 'mia_hand_mujoco/InteractiveSystemInterface',
    #         'enable_depth_publisher': 'false',
    #         'enable_preshaping_service': 'false',
    #         'scene': 'static',
    #         'include_wrist': 'true',
    #         'depth_publish_tf': 'false',
    #     }.items()
    # )
    #
    # For now, you can run the simulation manually:
    #   ros2 run mia_hand_mujoco interactive_system_interface_node

    # -- 2. Pointcloud relay -----------------------------------------------
    pc_relay = TimerAction(period=2.0, actions=[
        Node(
            package='camera',
            executable='pointcloud_relay_node',
            name='pointcloud_relay',
            output='screen',
        )
    ])

    # -- 3. Segmentation node ----------------------------------------------
    segmentation = TimerAction(period=3.0, actions=[
        Node(
            package='segmentation_bridge',
            executable='segmentation_ros2_node',
            name='segmentation_node',
            parameters=[{
                'cubeedge': segmentation_cubeedge,
                'inference_url': inference_url,
            }],
            output='screen',
        )
    ])

    # -- 4. Click relay ----------------------------------------------------
    click_relay = TimerAction(period=3.0, actions=[
        Node(
            package='segmentation_bridge',
            executable='demo_click_relay_node',
            name='click_relay',
            output='screen',
        )
    ])

    # -- 5. Grasp preshaping service bridge --------------------------------
    preshaping_bridge = TimerAction(period=5.0, actions=[
        Node(
            package='grasp_preshaping',
            executable='preshaping_service_bridge_node',
            name='preshaping_service_bridge',
            parameters=[{
                'camera_frames': ['cam1_d435_1_color_optical_frame', 'cam2_d435_2_color_optical_frame'],
                'preshaping_closure_fraction': 0.3,
                'min_closure_amount': 0.1,
                'publish_initial_commands': publish_initial_commands,
            }],
            output='screen',
        )
    ])

    # -- 6. Grasp proximity controller -------------------------------------
    proximity_controller = TimerAction(period=5.0, actions=[
        Node(
            package='grasp_preshaping',
            executable='grasp_proximity_controller_node.py',
            name='grasp_proximity_controller',
            parameters=[{
                'proximity_enter_threshold_m': 0.08,
                'proximity_exit_threshold_m': 0.10,
                'partial_closure_factor': 0.3,
                'min_closure_amount': 0.1,
                'control_rate_hz': 10.0,
            }],
            output='screen',
        )
    ])

    # -- 7. RViz -----------------------------------------------------------
    rviz_config = os.path.join(
        os.path.dirname(__file__), '..', '..', '..', '..', 'rviz', 'digital_twin.rviz'
    )
    rviz = TimerAction(period=8.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen',
        )
    ])

    # -- 8. Optional hand trajectory test node -----------------------------
    trajectory_test = TimerAction(period=15.0, actions=[
        Node(
            package='camera',
            executable='pointcloud_relay_node',
            name='hand_trajectory_test',
            output='screen',
            condition=IfCondition(use_trajectory),
        )
    ])
    # NOTE: The original hand trajectory test node (mujoco_hand_trajectory_node.py)
    # was part of the MuJoCo interactive simulator not yet ported. Replace the
    # Node above with the actual trajectory node once ported.

    return LaunchDescription([
        use_trajectory_arg,
        inference_url_arg,
        segmentation_cubeedge_arg,
        publish_initial_commands_arg,
        # mia_hand_sim_launch,   # uncomment when mia_hand_mujoco is ported
        pc_relay,
        segmentation,
        click_relay,
        preshaping_bridge,
        proximity_controller,
        rviz,
        trajectory_test,
    ])
