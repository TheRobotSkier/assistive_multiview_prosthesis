#!/usr/bin/env python3
"""Digital Twin launch file.

Orchestrates the full digital twin pipeline:
  1. Mia Hand MuJoCo simulation (InteractiveSystemInterface)
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
"""

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
    mia_hand_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mia_hand_mujoco'), 'launch',
                'mia_hand_system_interface_launch.py'
            ])
        ]),
        launch_arguments={
            'hardware_plugin': 'mia_hand_mujoco/InteractiveSystemInterface',
            'enable_depth_publisher': 'false',
            'enable_preshaping_service': 'false',
            'scene': 'static',
            'include_wrist': 'true',
            'depth_publish_tf': 'false',
        }.items()
    )

    # -- 2. Pointcloud relay -----------------------------------------------
    pc_relay = TimerAction(period=2.0, actions=[
        Node(
            executable='python3',
            arguments=['/miahand_ws/src/dev/mujoco/nodes/pointcloud_relay_node.py'],
            name='pointcloud_relay',
            output='screen',
        )
    ])

    # -- 3. Segmentation node ----------------------------------------------
    segmentation = TimerAction(period=3.0, actions=[
        Node(
            executable='python3',
            arguments=['/miahand_ws/src/dev/pc_segmentation/nodes/segmentation_ros2_node.py'],
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
            executable='python3',
            arguments=['/miahand_ws/src/dev/pc_segmentation/nodes/demo_click_relay_node.py'],
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
            executable='python3',
            arguments=['/miahand_ws/src/dev/grasp_preshaping/nodes/grasp_proximity_controller_node.py'],
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
    rviz = TimerAction(period=8.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', '/miahand_ws/src/dev/mujoco/config/digital_twin.rviz'],
            output='screen',
        )
    ])

    # -- 8. Optional hand trajectory test node -----------------------------
    trajectory_test = TimerAction(period=15.0, actions=[
        Node(
            executable='python3',
            arguments=['/miahand_ws/src/dev/mujoco/nodes/mujoco_hand_trajectory_node.py'],
            name='hand_trajectory_test',
            output='screen',
            condition=IfCondition(use_trajectory),
        )
    ])

    return LaunchDescription([
        use_trajectory_arg,
        inference_url_arg,
        segmentation_cubeedge_arg,
        publish_initial_commands_arg,
        mia_hand_sim_launch,
        pc_relay,
        segmentation,
        click_relay,
        preshaping_bridge,
        proximity_controller,
        rviz,
        trajectory_test,
    ])
