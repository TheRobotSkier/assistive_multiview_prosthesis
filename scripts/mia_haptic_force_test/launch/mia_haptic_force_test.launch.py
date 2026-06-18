"""Launch file for the multi-node mia_haptic_force_test stack.

All nodes are standalone scripts under ``scripts/mia_haptic_force_test/``.
This launch file lives next to those scripts and is designed to be invoked
either directly with ``ros2 launch`` from an installed package, or copied
into ``prosthesis_launch`` as the multi-node entry point.
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


def _as_bool(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).lower()
    return value in ("1", "true", "yes", "on")


def _node_process(name: str, output: str = "log") -> ExecuteProcess:
    script = os.path.join(_SCRIPT_DIR, f"{name}.py")
    return ExecuteProcess(
        cmd=["python3", "-m", f"scripts.mia_haptic_force_test.{name}"],
        name=name,
        output=output,
        sigkill_timeout="5",
        sigterm_timeout="3",
        env={"PYTHONPATH": _SCRIPT_DIR},
    )


def _launch_setup(context, *args, **kwargs):
    config_path = LaunchConfiguration("config_path").perform(context)
    log_level = LaunchConfiguration("log_level").perform(context)
    emg_enable = _as_bool(context, "emg_enable")
    haptic_enable = _as_bool(context, "haptic_enable")
    terminal_ui = _as_bool(context, "terminal_ui")
    logger = _as_bool(context, "logger")

    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Mia haptic force test config not found: {config_path}")

    nodes = []

    # Sensor / input nodes
    if emg_enable:
        nodes.append(_node_process("emg_input_node", output="screen"))
    nodes.append(_node_process("force_input_node", output="log"))

    # Control + supervisor
    nodes.append(_node_process("supervisor_node", output="screen"))
    nodes.append(_node_process("hand_controller_node", output="screen"))

    # Feedback / logging / UI
    if haptic_enable:
        nodes.append(_node_process("haptic_node", output="log"))
    if logger:
        nodes.append(_node_process("logger_node", output="log"))
    if terminal_ui:
        nodes.append(_node_process("terminal_ui_node", output="screen"))

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
