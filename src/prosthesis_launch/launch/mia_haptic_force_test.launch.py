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


def _resolve_port(context, name: str, env_var: str, default: str) -> str:
    """Resolve a device-port launch arg, falling back to the env var and
    then a hard default when the launch arg is empty.

    Prevents the xacro defaults (``/dev/ttyUSB0`` / ``/dev/ttyUSB1``) from
    leaking into the URDF when the upstream env-var chain is broken
    (e.g. ``podman exec -e KEY=""`` unsets the variable, the shell script
    then falls back to its own hard-coded default, and the launch arg
    override is an empty string that ``xacro`` resolves to its own default).
    """
    value = LaunchConfiguration(name).perform(context).strip()
    if value:
        return value
    env_value = os.environ.get(env_var, "").strip()
    if env_value:
        return env_value
    return default


def _load_yaml(path: str) -> dict:
    if not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        loaded = yaml.safe_load(f) or {}
    return loaded if isinstance(loaded, dict) else {}


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("config_path").perform(context)
    emg_model_dir = LaunchConfiguration("emg_model_dir").perform(context)
    mia_port = _resolve_port(context, "mia_port", "MIA_SERIAL_PORT", "/dev/ttyMiaHand")
    wrist_port = _resolve_port(context, "wrist_port", "WRIST_SERIAL_PORT", "/dev/ttyDynamixel")
    wrist_enable = _as_bool(context, "wrist_enable")
    haptic_enable = _as_bool(context, "haptic_enable")
    emg_enable = _as_bool(context, "emg_enable")
    mock_hardware = _as_bool(context, "mock_hardware")
    log_level = LaunchConfiguration("log_level").perform(context)
    use_multi_node = _as_bool(context, "use_multi_node")
    keyboard_emg = _as_bool(context, "keyboard_emg")
    terminal_ui = _as_bool(context, "terminal_ui")
    logger_enable = _as_bool(context, "logger")
    emg_board_ip = os.environ.get("EMG_BOARD_IP", "")

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Mia haptic force test config not found: {config_path}")
    # Keyboard emulation replaces the live EMG classifier; no model dir needed.
    if emg_enable and not keyboard_emg and not os.path.isdir(emg_model_dir):
        raise FileNotFoundError(
            f"EMG model directory not found: {emg_model_dir}. "
            "Run scripts/mia_haptic_force_test.sh so data/model setup happens first."
        )

    config = _load_yaml(config_path)
    wrist_cfg = config.get("wrist", {}) if isinstance(config.get("wrist", {}), dict) else {}

    nodes = []

    if use_multi_node:
        # New split-node stack (scripts/mia_haptic_force_test/)
        # Keyboard emulation, when enabled, replaces the live EMG input node
        # so the test runs without the MindRove bracelet.
        if keyboard_emg:
            emg_node_name = "keyboard_emg_node"
            emg_node_enabled = True
        else:
            emg_node_name = "emg_input_node"
            emg_node_enabled = emg_enable
        multi_nodes = [
            (emg_node_name, emg_node_enabled),
            ("force_input_node", True),
            ("supervisor_node", True),
            ("hand_controller_node", True),
            ("haptic_node", haptic_enable),
            ("logger_node", logger_enable),
            ("terminal_ui_node", terminal_ui),
        ]
        # In mock mode, launch the hand/wrist simulator BEFORE the
        # input nodes so that /hand_sim/joint_states and /wrist/state
        # are already publishing when force_input_node subscribes.
        if mock_hardware:
            multi_nodes.insert(0, ("hand_simulator_node", True))
        for name, enabled in multi_nodes:
            if not enabled:
                continue
            # Inherit the full parent launch process environment
            # (LD_LIBRARY_PATH, PATH, AMENT_PREFIX_PATH, ...).  An ExecuteProcess
            # `env` dict REPLACES the child environment, so passing only
            # {"PYTHONPATH": ...} strips LD_LIBRARY_PATH and rclpy's C
            # extension then dies with:
            #   ImportError: librcl_action.so: cannot open shared object file
            # We copy the parent env and only override PYTHONPATH so the
            # scripts/ package is importable alongside the ROS 2 site-packages.
            _child_env = os.environ.copy()
            cmd = [
                "python3",
                "-m",
                f"scripts.mia_haptic_force_test.{name}",
                "--config-path",
                config_path,
                "--ros-args",
                "--log-level",
                log_level,
            ]
            # In mock mode, point force_input_node at the simulator's
            # joint-state topic.  force_input_node does NOT read this
            # from the YAML `topics:` section, so it must be passed
            # via the command line.  We do NOT override
            # force_data_topic because the simulator publishes the
            # canonical ForceData on data_streams/fingers/forces/data
            # (the default), and /hand_sim/forces is a Float32MultiArray
            # passthrough that would cause a type-hash mismatch.
            if mock_hardware and name == "force_input_node":
                cmd += [
                    "-p", "joint_states_topic:=/hand_sim/joint_states",
                ]
            nodes.append(
                ExecuteProcess(
                    cmd=cmd,
                    name=name,
                    output="screen" if name in ("emg_input_node", "keyboard_emg_node", "supervisor_node", "hand_controller_node", "hand_simulator_node", "terminal_ui_node") else "log",
                    sigkill_timeout="5",
                    sigterm_timeout="3",
                    env=_child_env,
                )
            )
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

    if keyboard_emg:
        # Keyboard emulation replaces the live EMG classifier in the legacy
        # monolithic path too, so the toggle is not half-wired.
        # Same env fix as the multi-node path: inherit parent env so
        # LD_LIBRARY_PATH survives and rclpy can find its C libs.
        _child_env = os.environ.copy()
        _child_env["PYTHONPATH"] = "/prosthesis_ws/scripts:" + _child_env.get("PYTHONPATH", "")
        nodes.append(
            ExecuteProcess(
                cmd=[
                    "python3",
                    "-m",
                    "scripts.mia_haptic_force_test.keyboard_emg_node",
                    "--config-path",
                    config_path,
                    "--ros-args",
                    "--log-level",
                    log_level,
                ],
                name="keyboard_emg_node",
                sigkill_timeout="5",
                sigterm_timeout="3",
                env=_child_env,
            )
        )
    elif emg_enable:
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
            DeclareLaunchArgument(
                "keyboard_emg",
                default_value="false",
                description=(
                    "Replace the live EMG bracelet with keyboard arrow-key "
                    "emulation (←OPEN →POWER ↓FLEXION ↑EXTENSION, release→REST). "
                    "Requires a TTY for the keyboard_emg_node."
                ),
            ),
            DeclareLaunchArgument(
                "terminal_ui",
                default_value="true",
                description=(
                    "Launch terminal_ui_node (split-node only).  Set to "
                    "false for headless CI runs."
                ),
            ),
            DeclareLaunchArgument(
                "logger",
                default_value="true",
                description=(
                    "Launch logger_node (split-node only).  Set to false "
                    "for headless CI runs."
                ),
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
