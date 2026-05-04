"""
Launch mujoco_scene_state_publisher in hardware-mirror mode.

Visualises the live Mia Hand pose from /joint_states without any
ros2_control infrastructure. The node uses build_no_plugin_scene()
to strip actuator plugins from the XML automatically.

Set MUJOCO_SCENE env var to override the default scene path.
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node

_SCENE = os.environ.get(
    'MUJOCO_SCENE',
    '/miahand_ws/src/dev/mujoco/scenes/scene_right_static.xml',
)
_NODE_SCRIPT = '/miahand_ws/src/dev/mujoco/nodes/mujoco_scene_state_publisher_node.py'


def generate_launch_description():
    params = {
        'xml_model_path': _SCENE,
        'wait_for_joint_state': True,
        # Only include parameters that exist in the node:
        'publish_depth_image': False,
    }
    return LaunchDescription([
        Node(
            executable='python3',
            arguments=[_NODE_SCRIPT],
            name='mia_hand_mirror',
            parameters=[params],
            output='screen',
        )
    ])
