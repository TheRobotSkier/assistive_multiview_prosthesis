"""EMG Grasp Test Runtime Launch — minimal hardware control graph.

Launches ONLY what is needed for EMG-driven grasp testing:
  - Mia Hand ros2_control (position + velocity controllers)
  - Mia safety node (emergency stop / play services)
  - EMG classifier/bridge (MindRove → /emg/* topics)
  - EMG grasp controller node (state machine driving controllers)
  - (Optional) Wrist driver
  - Safety preflight checks
  - Status publisher

Explicitly does NOT launch:
  - Camera (RealSense / mock cloud)
  - Segmentation bridge
  - Pipeline manager
  - Production force_controller
  - Pointcloud fusion

Usage:
  ros2 launch prosthesis_launch emg_grasp_test.launch.py
  ros2 launch prosthesis_launch emg_grasp_test.launch.py \
    config_path:=/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml \
    emg_model_dir:=/app/models \
    mia_port:=/dev/ttyUSB0 \
    wrist_enable:=true \
    mock_hardware:=true

Arguments are also readable from config/emg_grasp_launch.yaml.
"""

import os
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# ── Defaults loaded from convenience config ────────────────────────────────────
_WORKSPACE_ROOT = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."
)
_DEFAULTS_PATH = os.path.join(_WORKSPACE_ROOT, "config", "emg_grasp_launch.yaml")
_DEFAULTS = {}
if os.path.exists(_DEFAULTS_PATH):
    with open(_DEFAULTS_PATH, "r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f) or {}
        if isinstance(loaded, dict):
            _DEFAULTS = {k: str(v) for k, v in loaded.items()}

_TESTS_ROOT = os.path.join(_WORKSPACE_ROOT, "tests", "emg_grasp")


def _as_bool(context, name: str) -> bool:
    """Evaluate a launch argument as a boolean."""
    value = LaunchConfiguration(name).perform(context).lower()
    return value in ("1", "true", "yes", "on")


def _launch_setup(context, *args, **kwargs):
    """Build the minimal EMG grasp test node list."""

    config_path = LaunchConfiguration("config_path").perform(context)
    emg_model_dir = LaunchConfiguration("emg_model_dir").perform(context)
    mia_port = LaunchConfiguration("mia_port").perform(context)
    wrist_enable = _as_bool(context, "wrist_enable")
    mock_hardware = _as_bool(context, "mock_hardware")
    log_level = LaunchConfiguration("log_level").perform(context)

    # ── Validate critical inputs ───────────────────────────────────────────────
    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"EMG grasp config not found: {config_path}\n"
            f"  Create one from {os.path.join(_TESTS_ROOT, 'emg_grasp_test.yaml')}\n"
            f"  or pass config_path:=/path/to/your/config.yaml"
        )

    if not os.path.isdir(emg_model_dir):
        raise FileNotFoundError(
            f"EMG model directory not found: {emg_model_dir}\n"
            f"  Train a model first:\n"
            f"    ros2 run emg_bridge train --data-dir /app/data --model-dir {emg_model_dir}\n"
            f"  Or pass emg_model_dir:=/path/to/trained/models"
        )

    nodes = []

    # ── 1. Mia Hand ros2_control (controller_manager + all controllers) ────────
    # Reuses the static_grasp_test controller-manager startup: spawns individual
    # per-finger position and velocity controllers for force-aware grasp control.
    # Position controllers: active (accept /commands)
    # Velocity controllers: inactive (activated by grasp node when needed)
    nodes.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("mia_hand_ros2_control"),
                    "launch",
                    "mia_hand_system_interface_launch.py",
                ])
            ]),
            launch_arguments={
                "serial_port": mia_port,
                "controller": "group_pos_ff_controller",
                "use_mock_hardware": "true" if mock_hardware else "false",
                "rviz2_gui": "false",
                "robot_ns": "mia_hand",
            }.items(),
        )
    )

    # ── 2. EMG classifier/bridge ───────────────────────────────────────────────
    # Connects to MindRove armband, runs trained classifier, publishes on
    #   /emg/gesture_label  (std_msgs/Int32,   latched)
    #   /emg/gesture_name   (std_msgs/String,  latched)
    #   /emg/confidence     (std_msgs/Float32, latched)
    #   /emg/proportional   (std_msgs/Float32, latched)
    nodes.append(
        ExecuteProcess(
            cmd=[
                "ros2",
                "run",
                "emg_bridge",
                "run_classifier",
                "--model-dir",
                emg_model_dir,
            ],
            name="emg_bridge",
            output="screen",
            sigkill_timeout="5",
            sigterm_timeout="3",
        )
    )

    # ── 3. EMG force-grasp bridge ──────────────────────────────────────────────
    # Owns the actual bridge between the live EMG classifier and hardware:
    # three-mode state machine, ros2_control hand switching, force hold, and
    # bounded wrist position increments.
    bridge_script = os.environ.get(
        "EMG_FORCE_GRASP_BRIDGE",
        "/prosthesis_ws/scripts/emg_force_grasp_bridge.py",
    )
    if not os.path.exists(bridge_script):
        raise FileNotFoundError(f"EMG force-grasp bridge not found: {bridge_script}")
    nodes.append(
        ExecuteProcess(
            cmd=[
                "python3",
                bridge_script,
                "--ros-args",
                "--log-level", log_level,
                "-p", f"config_path:={config_path}",
            ],
            output="screen",
            sigkill_timeout="5",
            sigterm_timeout="3",
        )
    )

    # ── 4. Optional wrist driver ───────────────────────────────────────────────
    if wrist_enable:
        wrist_port = LaunchConfiguration("wrist_port").perform(context)
        nodes.append(
            Node(
                package="wrist_driver",
                executable="wrist_driver_node",
                name="wrist_driver",
                parameters=[{"port": wrist_port}],
                output="screen",
            )
        )

    # ── 5. Safety preflight ────────────────────────────────────────────────────
    # Verifies controller_manager is reachable and joint_states are flowing
    # before the grasp controller begins issuing commands.
    # mia_safety_node is already started by mia_hand_system_interface_launch.
    preflight_script = os.path.join(_TESTS_ROOT, "emg_preflight.py")
    if os.path.exists(preflight_script):
        nodes.append(
            ExecuteProcess(
                cmd=[
                    "python3", preflight_script,
                    "--config", config_path,
                    "--mia-port", mia_port,
                    "--mock", "true" if mock_hardware else "false",
                    "--timeout", "15",
                ],
                name="emg_preflight",
                output="screen",
            )
        )
    else:
        # Fallback: brief wait for controller_manager to initialise
        # TODO(mvp-egt.8): Replace with proper preflight node
        nodes.append(
            ExecuteProcess(
                cmd=[
                    "python3", "-c",
                    "; ".join([
                        "import time",
                        "print('[preflight] Waiting for controller_manager to initialise …')",
                        "time.sleep(3)",
                        f"print('[preflight] Controller startup grace period elapsed (serial={mia_port}).')",
                    ]),
                ],
                name="emg_preflight_fallback",
                output="screen",
            )
        )

    # ── 6. Status publisher ────────────────────────────────────────────────────
    # Publishes system status on /emg/system_status for external monitoring
    # (e.g. the Makefile rule or a dashboard).
    status_script = "/prosthesis_ws/tests/emg_grasp/emg_status_publisher.py"
    if os.path.exists(status_script):
        nodes.append(
            ExecuteProcess(
                cmd=[
                    "python3", status_script,
                    "--ros-args", "--log-level", "warn",
                ],
                name="emg_status_publisher",
                output="screen",
            )
        )

    # ── 7. Cleanup on shutdown ─────────────────────────────────────────────────
    nodes.append(
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    ExecuteProcess(
                        cmd=[
                            "python3", "-c",
                            "print('[cleanup] EMG grasp test shut down.')",
                        ],
                        output="screen",
                    ),
                ]
            )
        )
    )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ───────────────────────────────────────────────────
        DeclareLaunchArgument(
            "config_path",
            default_value=_DEFAULTS.get(
                "config_path",
                os.path.join(_TESTS_ROOT, "emg_grasp_test.yaml"),
            ),
            description="Path to EMG grasp test YAML configuration.",
        ),
        DeclareLaunchArgument(
            "emg_model_dir",
            default_value=_DEFAULTS.get("emg_model_dir", "/prosthesis_ws/models"),
            description="Directory containing trained EMG classifier models.",
        ),
        DeclareLaunchArgument(
            "emg_device",
            default_value=_DEFAULTS.get("emg_device", ""),
            description="EMG device path (reserved for future use).",
        ),
        DeclareLaunchArgument(
            "mia_port",
            default_value=_DEFAULTS.get("mia_port", "/dev/ttyUSB0"),
            description="Serial port for the Mia Hand.",
        ),
        DeclareLaunchArgument(
            "wrist_port",
            default_value=_DEFAULTS.get("wrist_port", "/dev/ttyUSB1"),
            description="Serial port for the wrist Dynamixel motor.",
        ),
        DeclareLaunchArgument(
            "wrist_enable",
            default_value=_DEFAULTS.get("wrist_enable", "false"),
            description="Enable wrist Dynamixel driver (true/false).",
        ),
        DeclareLaunchArgument(
            "mock_hardware",
            default_value=_DEFAULTS.get("mock_hardware", "false"),
            description="Use mock hardware mirroring commands to states (true/false).",
        ),
        DeclareLaunchArgument(
            "log_level",
            default_value=_DEFAULTS.get("log_level", "info"),
            description="ROS 2 logging level (debug/info/warn/error/fatal).",
        ),

        # ── Build node list via OpaqueFunction ─────────────────────────────────
        OpaqueFunction(function=_launch_setup),
    ])
