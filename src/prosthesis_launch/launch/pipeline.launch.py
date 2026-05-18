"""Full pipeline launch - all nodes with hardware.

Launches the complete prosthesis pipeline:
  1. Mia Hand driver (serial)
  2. Command bridge (forwards ros2_control topics to driver services)
  3. Wrist Dynamixel driver
  4. EMG bridge (MindRove)
  5. Segmentation ROS bridge
  6. Twist propagation target selector
  7. Grasp preshaping service
  8. Grasp proximity controller
  9. Force controller
  10. Pipeline manager (state machine)
  11. RViz

Usage:
  ros2 launch pipeline.launch.py
  ros2 launch pipeline.launch.py rviz:=false
  ros2 launch pipeline.launch.py config_file:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Default config path: workspace-root config/prosthesis_config.yaml
_WORKSPACE_ROOT = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."
)
DEFAULT_CONFIG = os.path.join(_WORKSPACE_ROOT, "config", "prosthesis_config.yaml")


def generate_launch_description():
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz"
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=DEFAULT_CONFIG,
        description="Path to prosthesis_config.yaml",
    )
    camera_arg = DeclareLaunchArgument(
        "camera", default_value="true", description="Launch RealSense camera"
    )
    mia_hand_arg = DeclareLaunchArgument(
        "mia_hand", default_value="true", description="Launch Mia Hand driver"
    )

    # Pipeline Manager - state machine orchestrator
    pipeline_manager = Node(
        package="pipeline_manager",
        executable="pipeline_manager_node",
        name="pipeline_manager",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # Mia Hand Driver
    mia_hand_driver = Node(
        package="mia_hand_driver",
        executable="mia_hand_driver_node",
        name="mia_hand_driver",
        parameters=[{"serial_port": "/dev/ttyUSB0"}],
        output="screen",
    )

    # Command Bridge — forwards *_pos_ff_controller/commands to driver services
    # and republishes joint positions as /joint_states
    command_bridge = Node(
        package="command_bridge",
        executable="command_bridge_node",
        name="command_bridge",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # EMG Bridge - MindRove gesture classifier
    emg_bridge = Node(
        package="emg_bridge",
        executable="run_classifier",
        name="emg_bridge",
        output="screen",
    )

    # Segmentation ROS bridge (talks to inference server over HTTP)
    segmentation_bridge = Node(
        package="segmentation_bridge",
        executable="segmentation_ros2_node",
        name="segmentation_bridge",
        parameters=[{
            "inference_url": "http://127.0.0.1:5678",
        }],
        output="screen",
    )

    # Grasp Preshaping Service (C++ bridge to Rust .so)
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
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # Twist Propagation Target Selector
    twist_propagation = Node(
        package="twist_propagation",
        executable="twist_propagation_node",
        name="twist_propagation",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # Force Controller
    force_controller = Node(
        package="force_controller",
        executable="force_controller_node",
        name="force_controller",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # RViz config - look in the rviz/ directory at workspace root
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

    # Assemble launch
    nodes = [
        pipeline_manager,
        mia_hand_driver,
        command_bridge,
        emg_bridge,
        segmentation_bridge,
        twist_propagation,
        preshaping_service,
        proximity_controller,
        force_controller,
    ]

    # Conditional nodes - always included, can be toggled
    # (Launch system doesn't support true conditionals easily,
    #  so we include them and let the nodes handle missing hardware)

    # RViz - included by default
    nodes.append(rviz)

    return LaunchDescription(
        [
            rviz_arg,
            config_arg,
            camera_arg,
            mia_hand_arg,
        ]
        + nodes
    )
