from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # ── launch arguments ──────────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Connection identifier for the EMG armband (serial port, BT address, …)",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("emg_armband"), "config", "emg.yaml"]
        ),
        description="Path to emg.yaml configuration file",
    )

    # ── emg_node — raw hardware interface ────────────────────────────────────
    emg_node = Node(
        name="emg_node",
        package="emg_armband",
        executable="emg_node.py",
        output="screen",
        parameters=[
            {
                "serial_port": LaunchConfiguration("serial_port"),
                "config_file": LaunchConfiguration("config_file"),
            }
        ],
    )

    # ── emg_parser_node — config-driven signal routing ───────────────────────
    emg_parser_node = Node(
        name="emg_parser_node",
        package="emg_armband",
        executable="emg_parser_node.py",
        output="screen",
        parameters=[
            {
                "config_file": LaunchConfiguration("config_file"),
            }
        ],
    )

    return LaunchDescription(
        [
            serial_port_arg,
            config_file_arg,
            emg_node,
            emg_parser_node,
        ]
    )
