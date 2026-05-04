#!/usr/bin/env python3
"""Full system test: MuJoCo mirror + haptic controller + segmentation + RViz.

Services to start first (see manual startup order in plan):
  docker compose run --rm miahand_driver
  docker compose run --rm haptic_band
  docker compose run --rm multiview_full
  docker compose run --rm mindrove_emg_ros
  docker compose run --rm full_system_test   
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

_SCENE = os.environ.get(
    'MUJOCO_SCENE',
    '/miahand_ws/src/dev/mujoco/scenes/scene_right_static.xml',
)
_RVIZ_CONFIG = '/miahand_ws/src/dev/mujoco/config/full_system_test.rviz'
_NODE_SCRIPT = '/miahand_ws/src/dev/mujoco/nodes/mujoco_scene_state_publisher_node.py'
_SEG_SCRIPT = '/miahand_ws/src/dev/pc_segmentation/nodes/segmentation_ros_node.py'


def generate_launch_description():
    mujoco_mirror = Node(
        executable='python3',
        arguments=[_NODE_SCRIPT],
        name='mia_hand_mirror',
        parameters=[{
            'xml_model_path': _SCENE,
            'wait_for_joint_state': True,
            'publish_depth_image': False,
        }],
        output='screen',
    )

    haptic_controller = Node(
        package='haptic_bridge',
        executable='haptic_controller_node',
        name='haptic_controller',
        output='screen',
    )

    segmentation = Node(
        executable='python3',
        arguments=[_SEG_SCRIPT],
        name='segmentation_node',
        output='screen',
    )

    rviz = TimerAction(period=5.0, actions=[
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', _RVIZ_CONFIG],
            output='screen',
        )
    ])

    return LaunchDescription([mujoco_mirror, haptic_controller, segmentation, rviz])
