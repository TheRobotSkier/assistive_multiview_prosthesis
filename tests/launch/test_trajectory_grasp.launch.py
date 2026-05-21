"""Smoke-test launch file for trajectory-selected segmentation grasp pipeline.

Brings up the core pipeline nodes alongside a mock data publisher so the
system can be exercised without real hardware or inference servers.

Usage:
    ros2 launch tests/launch/test_trajectory_grasp.launch.py

Nodes started:
  - pipeline_manager
  - twist_propagation
  - grasp_proximity_controller
  - mock_data_publisher (publishes hand_pose, cloud, EMG gestures)
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    mock_script_path = \
        'tests/launch/mock_data_publisher.py'

    return LaunchDescription([
        # Mock data sources (run as standalone Python script)
        ExecuteProcess(
            cmd=['python3', mock_script_path],
            output='screen',
        ),

        # Pipeline orchestration
        Node(
            package='pipeline_manager',
            executable='pipeline_manager_node.py',
            name='pipeline_manager',
            output='screen',
            parameters=[{
                'confidence_threshold': 0.5,
                'grasp_gestures': [1, 2, 4],  # POWER, PINCH, POINT
            }],
        ),

        # Twist propagation (autonomous target selection)
        Node(
            package='twist_propagation',
            executable='twist_propagation_node.py',
            name='twist_propagation',
            output='screen',
            parameters=[{
                'active': False,  # activated by pipeline_manager
                'assert_twist_frames_match': False,
            }],
        ),

        # Proximity controller
        Node(
            package='grasp_preshaping',
            executable='grasp_proximity_controller_node.py',
            name='grasp_proximity_controller',
            output='screen',
        ),
    ])
