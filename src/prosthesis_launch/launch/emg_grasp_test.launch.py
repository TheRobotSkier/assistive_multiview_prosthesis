"""Launch file for EMG-driven velocity-based force grasp test.

Launches:
  - Mia Hand ros2_control (with individual position + velocity controllers)
  - Mia safety node
  - EMG grasp test node

Usage:
  ros2 launch prosthesis_launch emg_grasp_test.launch.py
  ros2 launch prosthesis_launch emg_grasp_test.launch.py emg:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    emg_arg = DeclareLaunchArgument(
        "emg", default_value="false", description="Launch EMG bridge node"
    )
    mock_emg_arg = DeclareLaunchArgument(
        "mock_emg",
        default_value="false",
        description="Launch mock EMG publisher (gesture=1, duration=5s)",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Mia Hand",
    )
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml",
        description="Path to EMG grasp test YAML config",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Use mock Mia hand hardware (no physical device needed)",
    )

    emg = LaunchConfiguration("emg")
    mock_emg = LaunchConfiguration("mock_emg")
    serial_port = LaunchConfiguration("serial_port")
    config_path = LaunchConfiguration("config_path")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Mia Hand system interface — spawns all individual pos+vel controllers
    mia_hand_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("mia_hand_ros2_control"),
                "launch",
                "mia_hand_system_interface_launch.py",
            ])
        ]),
        launch_arguments={
            "serial_port": serial_port,
            "controller": "",
            "use_mock_hardware": use_mock_hardware,
            "rviz2_gui": "false",
        }.items(),
    )

    # EMG grasp test node (lives in tests/ so we use ExecuteProcess)
    emg_grasp_node = ExecuteProcess(
        cmd=[
            "python3",
            "/prosthesis_ws/tests/emg_grasp/emg_grasp_node.py",
            "--ros-args",
            "-p",
            ["config_path:=", config_path],
        ],
        output="screen",
    )

    # Mock EMG publisher (for non-hardware testing)
    mock_emg_node = GroupAction(
        condition=IfCondition(mock_emg),
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    "/prosthesis_ws/tests/emg_grasp/mock_emg_publisher.py",
                    "--gesture", "1",
                    "--confidence", "0.95",
                    "--duration", "5.0",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        emg_arg,
        mock_emg_arg,
        serial_port_arg,
        config_path_arg,
        use_mock_hardware_arg,
        mia_hand_launch,
        emg_grasp_node,
        mock_emg_node,
    ])
