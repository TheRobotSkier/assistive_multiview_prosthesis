"""Simplified EMG-triggered grasp pipeline -- no perception.

Launches only the execution layer:
   1. Mia Hand driver (serial)
   2. Command bridge (forwards ros2_control topics to driver services)
   3. Wrist Dynamixel driver
   4. EMG bridge (MindRove classifier)
   5. Force controller
   6. Simple pipeline manager (EMG-triggered state machine)
   7. RViz (optional)

Unlike the full pipeline, this does NOT launch:
   - Pointcloud fusion, camera TF bridges, odometry relays
   - Segmentation bridge
   - Twist propagation
   - Grasp preshaping service
   - Proximity controller

Grasp is triggered directly by EMG POWER gesture (held >= 1s).
The wrist preshapes concurrently with force controller closure.

Usage:
  ros2 launch prosthesis_launch simple_emg_grasp.launch.py
  ros2 launch prosthesis_launch simple_emg_grasp.launch.py rviz:=false
  ros2 launch prosthesis_launch simple_emg_grasp.launch.py wrist:=false
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_CONFIG = os.environ.get(
    "PROSTHESIS_CONFIG",
    "/prosthesis_ws/config/prosthesis_config.yaml",
)
DEFAULT_RVIZ_CONFIG = os.environ.get(
    "PROSTHESIS_RVIZ_CONFIG",
    "/prosthesis_ws/rviz/prosthesis.rviz",
)


def _as_bool(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).lower()
    return value in ("1", "true", "yes", "on")


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _node_params(config: dict, node_name: str) -> dict:
    section = config.get(node_name, {})
    if not isinstance(section, dict):
        return {}
    ros_params = section.get("ros__parameters")
    if isinstance(ros_params, dict):
        return dict(ros_params)
    return dict(section)


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_file)

    mia_serial_port = LaunchConfiguration("mia_serial_port").perform(context)
    wrist_serial_port = LaunchConfiguration("wrist_serial_port").perform(context)
    model_dir = LaunchConfiguration("model_dir").perform(context)

    nodes = []

    # Simple Pipeline Manager -- EMG-triggered state machine
    nodes.append(
        Node(
            package="pipeline_manager",
            executable="simple_pipeline_manager_node",
            name="simple_pipeline_manager",
            parameters=[_node_params(config, "simple_pipeline_manager")],
            output="screen",
        )
    )

    if _as_bool(context, "mia_hand"):
        nodes.extend(
            [
                Node(
                    package="mia_hand_driver",
                    executable="mia_hand_driver_node",
                    name="mia_hand_driver",
                    parameters=[{"serial_port": mia_serial_port}],
                    output="screen",
                ),
                Node(
                    package="command_bridge",
                    executable="command_bridge_node",
                    name="command_bridge",
                    parameters=[_node_params(config, "command_bridge")],
                    output="screen",
                ),
            ]
        )

    if _as_bool(context, "wrist"):
        hardware = config.get("hardware", {}) if isinstance(config.get("hardware", {}), dict) else {}
        nodes.append(
            Node(
                package="wrist_driver",
                executable="wrist_driver_node",
                name="wrist_driver",
                parameters=[
                    {
                        "port": wrist_serial_port,
                        "baudrate": hardware.get("wrist_baudrate", 57600),
                        "motor_id": hardware.get("wrist_motor_id", 1),
                    }
                ],
                output="screen",
            )
        )

    if _as_bool(context, "emg"):
        nodes.append(
            Node(
                package="emg_bridge",
                executable="run_classifier",
                name="emg_bridge",
                arguments=["--model-dir", model_dir],
                output="screen",
            )
        )

    if _as_bool(context, "mia_hand"):
        nodes.append(
            Node(
                package="force_controller",
                executable="force_controller_node",
                name="force_controller",
                parameters=[_node_params(config, "force_controller")],
                output="screen",
            )
        )

    if _as_bool(context, "rviz"):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", DEFAULT_RVIZ_CONFIG],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Launch RViz"
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=DEFAULT_CONFIG,
                description="Path to prosthesis_config.yaml",
            ),
            DeclareLaunchArgument(
                "mia_hand",
                default_value="true",
                description="Launch Mia Hand driver and force controller",
            ),
            DeclareLaunchArgument(
                "wrist", default_value="true", description="Launch wrist Dynamixel driver"
            ),
            DeclareLaunchArgument(
                "emg", default_value="true", description="Launch EMG classifier bridge"
            ),
            DeclareLaunchArgument(
                "mia_serial_port",
                default_value=os.environ.get("MIA_SERIAL_PORT", "/dev/ttyMiaHand"),
                description="Mia Hand serial port device",
            ),
            DeclareLaunchArgument(
                "wrist_serial_port",
                default_value=os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyDynamixel"),
                description="Wrist Dynamixel serial port device",
            ),
            DeclareLaunchArgument(
                "model_dir",
                default_value="/app/models",
                description="Directory containing trained EMG classifier models.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
