"""Mock pipeline launch - all nodes with simulated data.

Launches the prosthesis pipeline without hardware:
  - No Mia Hand serial connection
  - No MindRove EMG band
  - No RealSense camera
  - Mock publishers simulate sensor data

Use this for development, testing, and debugging the pipeline logic.

Usage:
  ros2 launch mock.launch.py
  ros2 launch mock.launch.py config_file:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="Path to prosthesis_config.yaml (empty = defaults)",
    )

    # ── Mock data publishers ──────────────────────────────────────────────
    # Mock EMG gesture publisher (cycles through gestures)
    mock_emg = Node(
        package="pipeline_manager",
        executable="pipeline_manager_node",
        name="pipeline_manager",
        parameters=[{
            "config_file": LaunchConfiguration("config_file"),
            "mock_mode": True,
        }],
        output="screen",
    )

    # Mock point cloud publisher (generates synthetic object cloud)
    mock_cloud = Node(
        package="pipeline_manager",
        executable="mock_cloud_publisher",
        name="mock_cloud_publisher",
        output="screen",
    )

    # ── Pipeline nodes (same as real) ─────────────────────────────────────
    # Segmentation ROS bridge (can point to inference server or mock)
    segmentation_bridge = Node(
        package="segmentation_bridge",
        executable="segmentation_ros2_node",
        name="segmentation_bridge",
        parameters=[{
            "inference_url": "http://127.0.0.1:5678",
        }],
        output="screen",
    )

    # Grasp Preshaping Service
    preshaping_service = Node(
        package="grasp_preshaping",
        executable="preshaping_service_bridge_node",
        name="preshaping_service",
        output="screen",
    )

    # Grasp Proximity Controller
    proximity_controller = Node(
        package="grasp_preshaping",
        executable="grasp_proximity_controller_node.py",
        name="proximity_controller",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
        output="screen",
    )

    # Force Controller
    force_controller = Node(
        package="force_controller",
        executable="force_controller_node",
        name="force_controller",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
        output="screen",
    )

    # RViz
    rviz_config = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "rviz", "prosthesis.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            config_arg,
            mock_emg,
            preshaping_service,
            proximity_controller,
            force_controller,
            rviz,
        ]
    )
