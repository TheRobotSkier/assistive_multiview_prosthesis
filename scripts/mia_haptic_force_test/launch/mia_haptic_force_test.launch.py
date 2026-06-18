"""Launch file for the multi-node mia_haptic_force_test stack.

All nodes are standalone scripts under ``scripts/mia_haptic_force_test/``.
This launch file lives next to those scripts and is designed to be invoked
either directly with ``ros2 launch`` from an installed package, or copied
into ``prosthesis_launch`` as the multi-node entry point.

Each node receives ``--config-path <path>`` as a CLI argument so it can load
the shared YAML config without hardcoding a path.  Nodes parse this flag via
``argparse`` (node-side parsing is added by mvp-8uv.2).  Any unknown args
are forwarded to ``rclpy.init()`` so ROS 2 remapping still works.

Individual nodes can also be launched directly:

    python3 -m scripts.mia_haptic_force_test.emg_input_node \\
        --config-path /path/to/config.yaml
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


_DEFAULT_CONFIG = os.environ.get(
    "MIA_HAPTIC_FORCE_TEST_CONFIG",
    "/prosthesis_ws/config/mia_haptic_force_test.yaml",
)
_SCRIPT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_LAUNCH_DIR = os.path.dirname(os.path.abspath(__file__))


def _as_bool(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).lower()
    return value in ("1", "true", "yes", "on")


def _node_process(name: str, config_path: str, output: str = "log") -> ExecuteProcess:
    """Spawn a split-node script as a child process.

    Each node receives ``--config-path <path>`` so it can load the shared
    YAML config without hardcoding.  The node-side ``argparse`` handler
    (added by mvp-8uv.2) parses this flag; any remaining ROS args are
    forwarded unchanged.
    """
    return ExecuteProcess(
        cmd=["python3", "-m", f"scripts.mia_haptic_force_test.{name}",
             "--config-path", config_path],
        name=name,
        output=output,
        sigkill_timeout="5",
        sigterm_timeout="3",
        env={"PYTHONPATH": os.path.dirname(_SCRIPT_DIR)},
    )


def _force_input_with_sim_joint_states(config_path: str) -> ExecuteProcess:
    """Spawn force_input_node pointed at the simulator's joint-state topic.

    In ``mock_hardware=true`` mode the simulator publishes
    ``/hand_sim/joint_states`` instead of the real ``/joint_states`` (which
    the joint_state_broadcaster in ``mia_hand_system_interface_launch.py``
    already publishes in mock mode).  We pass the topic via ``-p
    joint_states_topic:=...`` on the command line because force_input_node
    reads the parameter from the ROS parameter server, not from the YAML
    ``topics:`` section.
    """
    return ExecuteProcess(
        cmd=[
            "python3", "-m", "scripts.mia_haptic_force_test.force_input_node",
            "--config-path", config_path,
            "--ros-args", "-p", "joint_states_topic:=/hand_sim/joint_states",
            "-p", "force_data_topic:=/hand_sim/forces",
        ],
        name="force_input_node",
        output="log",
        sigkill_timeout="5",
        sigterm_timeout="3",
        env={"PYTHONPATH": os.path.dirname(_SCRIPT_DIR)},
    )


def _simulator_process(config_path: str) -> ExecuteProcess:
    """Spawn the hand/wrist simulator backend."""
    return ExecuteProcess(
        cmd=["python3", "-m", "scripts.mia_haptic_force_test.hand_simulator_node",
             "--config-path", config_path],
        name="hand_simulator_node",
        output="log",
        sigkill_timeout="5",
        sigterm_timeout="3",
        env={"PYTHONPATH": os.path.dirname(_SCRIPT_DIR)},
    )


def _launch_setup(context, *args, **kwargs):  # noqa: ARG001
    """Build the list of ``ExecuteProcess`` actions for enabled nodes.

    Reads launch arguments, validates ``config_path`` exists on disk, then
    returns one ``ExecuteProcess`` per enabled node — each receiving
    ``--config-path <path>`` so it can load the shared YAML config.
    """
    config_path = LaunchConfiguration("config_path").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    mock_hardware = _as_bool(context, "mock_hardware")
    emg_enable = _as_bool(context, "emg_enable") and not mock_hardware
    haptic_enable = _as_bool(context, "haptic_enable")
    terminal_ui = _as_bool(context, "terminal_ui")
    logger = _as_bool(context, "logger")
    wrist_enable = _as_bool(context, "wrist_enable")

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Mia haptic force test config not found: {config_path}")

    nodes = []

    # Mock simulator (publishes raw hardware streams + /wrist/state)
    if mock_hardware:
        nodes.append(_simulator_process(config_path))

    # Sensor / input nodes
    if emg_enable:
        nodes.append(_node_process("emg_input_node", config_path, output="screen"))
    if mock_hardware:
        nodes.append(_force_input_with_sim_joint_states(config_path))
    else:
        nodes.append(_node_process("force_input_node", config_path, output="log"))

    # Control + supervisor
    nodes.append(_node_process("supervisor_node", config_path, output="screen"))
    nodes.append(_node_process("hand_controller_node", config_path, output="screen"))
    # Feedback / logging / UI
    if haptic_enable:
        nodes.append(_node_process("haptic_node", config_path, output="log"))
    if logger:
        nodes.append(_node_process("logger_node", config_path, output="log"))
    if terminal_ui:
        nodes.append(_node_process("terminal_ui_node", config_path, output="screen"))

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
                "log_level",
                default_value="info",
                description="ROS log level.",
            ),
            DeclareLaunchArgument(
                "mock_hardware",
                default_value="false",
                description=(
                    "Launch the software hand/wrist simulator instead of "
                    "expecting real hardware.  When true, the simulator "
                    "publishes the raw Mia hardware streams and /wrist/state."
                ),
            ),
            DeclareLaunchArgument(
                "wrist_enable",
                default_value="true",
                description=(
                    "Enable the real wrist driver.  Set to false in "
                    "mock_hardware=true mode so the real /dev/ttyDynamixel "
                    "is not opened."
                ),
            ),
            DeclareLaunchArgument(
                "emg_enable",
                default_value="true",
                description="Launch live EMG classifier node.",
            ),
            DeclareLaunchArgument(
                "haptic_enable",
                default_value="true",
                description="Launch haptic feedback node.",
            ),
            DeclareLaunchArgument(
                "terminal_ui",
                default_value="true",
                description="Launch terminal UI node.",
            ),
            DeclareLaunchArgument(
                "logger",
                default_value="true",
                description="Launch CSV logger node.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
