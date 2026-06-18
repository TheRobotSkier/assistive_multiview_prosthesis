"""Mia Hand EMG haptic force test launch.

Launches only the hardware/input/feedback pieces needed for the bench test:

- Mia Hand ros2_control with position and velocity controllers
- live EMG classifier
- optional wrist Dynamixel driver
- optional Vibro8 Bluetooth bridge
- the staged test node in scripts/mia_haptic_force_test.py

No cameras, segmentation, pointcloud fusion, preshaping, or production pipeline
manager are launched.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


_DEFAULT_CONFIG = os.environ.get(
    "MIA_HAPTIC_FORCE_TEST_CONFIG",
    "/prosthesis_ws/config/mia_haptic_force_test.yaml",
)


def _as_bool(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).lower()
    return value in ("1", "true", "yes", "on")


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f) or {}
    return loaded if isinstance(loaded, dict) else {}


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("config_path").perform(context)
    emg_model_dir = LaunchConfiguration("emg_model_dir").perform(context)
    mia_port = LaunchConfiguration("mia_port").perform(context)
    wrist_port = LaunchConfiguration("wrist_port").perform(context)
    wrist_enable = _as_bool(context, "wrist_enable")
    haptic_enable = _as_bool(context, "haptic_enable")
    emg_enable = _as_bool(context, "emg_enable")
    mock_hardware = _as_bool(context, "mock_hardware")
    log_level = LaunchConfiguration("log_level").perform(context)
    use_multi_node = _as_bool(context, "use_multi_node")
    emg_board_ip = os.environ.get("EMG_BOARD_IP", "")

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Mia haptic force test config not found: {config_path}")
    if emg_enable and not os.path.isdir(emg_model_dir):
        raise FileNotFoundError(
            f"EMG model directory not found: {emg_model_dir}. "
            "Run scripts/mia_haptic_force_test.sh so data/model setup happens first."
        )

    config = _load_yaml(config_path)
    wrist_cfg = config.get("wrist", {}) if isinstance(config.get("wrist", {}), dict) else {}

    nodes = []

    if use_multi_node:
        # New split-node stack (scripts/mia_haptic_force_test/)
        multi_nodes = [
            ("emg_input_node", emg_enable),
            ("force_input_node", True),
            ("supervisor_node", True),
            ("hand_controller_node", True),
            ("haptic_node", haptic_enable),
            ("logger_node", True),
            ("terminal_ui_node", True),
        ]
        for name, enabled in multi_nodes:
            if not enabled:
                continue
            nodes.append(
                ExecuteProcess(
                    cmd=["python3", "-m", f"scripts.mia_haptic_force_test.{name}"],
                    name=name,
                    output="screen" if name in ("emg_input_node", "supervisor_node", "hand_controller_node", "terminal_ui_node") else "log",
                    sigkill_timeout="5",
                    sigterm_timeout="3",
                    env={"PYTHONPATH": "/prosthesis_ws/scripts"},
                )
            )

        # Shared hardware drivers (same as legacy path)
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("mia_hand_ros2_control"),
                                "launch",
                                "mia_hand_system_interface_launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "serial_port": mia_port,
                    "controller": "group_pos_ff_controller",
                    "use_mock_hardware": "true" if mock_hardware else "false",
                    "rviz2_gui": "false",
                    "robot_ns": "mia_hand",
                }.items(),
            )
        )

        if wrist_enable:
            nodes.append(
                Node(
                    package="wrist_driver",
                    executable="wrist_driver_node",
                    name="wrist_driver",
                    parameters=[
                        {
                            "port": wrist_port,
                            "min_position_deg": wrist_cfg.get("min_deg", 5.0),
                            "max_position_deg": wrist_cfg.get("max_deg", 300.0),
                        }
                    ],
                    output="screen",
                )
            )

        if haptic_enable:
            nodes.append(
                Node(
                    package="haptic_bridge",
                    executable="bridge_node",
                    name="haptic_bridge_node",
                    output="screen",
                )
            )

        return nodes

    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("mia_hand_ros2_control"),
                            "launch",
                            "mia_hand_system_interface_launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                "serial_port": mia_port,
                "controller": "group_pos_ff_controller",
                "use_mock_hardware": "true" if mock_hardware else "false",
                "rviz2_gui": "false",
                "robot_ns": "mia_hand",
            }.items(),
        )
    )

    if emg_enable:
        nodes.append(
            Node(
                package="emg_bridge",
                executable="run_classifier",
                name="emg_bridge",
                arguments=["--model-dir", emg_model_dir] + ([f"--ip={emg_board_ip}"] if emg_board_ip else []),
            )
        )

    if wrist_enable:
        nodes.append(
            Node(
                package="wrist_driver",
                executable="wrist_driver_node",
                name="wrist_driver",
                parameters=[
                    {
                        "port": wrist_port,
                        "min_position_deg": wrist_cfg.get("min_deg", 5.0),
                        "max_position_deg": wrist_cfg.get("max_deg", 300.0),
                    }
                ],
                output="screen",
            )
        )

    if haptic_enable:
        nodes.append(
            Node(
                package="haptic_bridge",
                executable="bridge_node",
                name="haptic_bridge_node",
                output="screen",
            )
        )

    test_script = "/prosthesis_ws/scripts/mia_haptic_force_test.py"
    if not os.path.exists(test_script):
        raise FileNotFoundError(f"Test script not found: {test_script}")
    test_process = ExecuteProcess(
        cmd=[
            "python3",
            test_script,
            "--ros-args",
            "--log-level",
            log_level,
            "-p",
            f"config_path:={config_path}",
        ],
        name="mia_haptic_force_test",
        output="screen",
        sigkill_timeout="5",
        sigterm_timeout="3",
    )
    nodes.append(test_process)
    nodes.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=test_process,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(
                            reason="mia_haptic_force_test process exited"
                        )
                    )
                ],
            )
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config_path",
                default_value=_DEFAULT_CONFIG,
                description="Path to config/mia_haptic_force_test.yaml.",
            ),
            DeclareLaunchArgument(
                "emg_model_dir",
                default_value="/app/models",
                description="Directory containing trained EMG classifier models.",
            ),
            DeclareLaunchArgument(
                "mia_port",
                default_value=os.environ.get("MIA_SERIAL_PORT", "/dev/ttyMiaHand"),
                description="Mia Hand serial port.",
            ),
            DeclareLaunchArgument(
                "wrist_port",
                default_value=os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyDynamixel"),
                description="Wrist Dynamixel serial port.",
            ),
            DeclareLaunchArgument(
                "wrist_enable",
                default_value="true",
                description="Launch wrist driver.",
            ),
            DeclareLaunchArgument(
                "haptic_enable",
                default_value="true",
                description="Launch Vibro8 Bluetooth bridge.",
            ),
            DeclareLaunchArgument(
                "emg_enable",
                default_value="true",
                description="Launch live EMG classifier.",
            ),
            DeclareLaunchArgument(
                "mock_hardware",
                default_value="false",
                description="Use ros2_control mock hand hardware.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the test process.",
            ),
            DeclareLaunchArgument(
                "use_multi_node",
                default_value="false",
                description="Launch the new split-node stack instead of the legacy monolithic script.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
