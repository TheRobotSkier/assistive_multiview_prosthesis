"""Launch file for EMG volitional hand controller test.

Launches:
  - Mia Hand ros2_control with RViz (individual pos+vel controllers)
  - Mia safety node
  - EMG volitional controller node
  - Optional: mock EMG publisher

Usage:
  ros2 launch prosthesis_launch emg_volitional_test.launch.py
  ros2 launch prosthesis_launch emg_volitional_test.launch.py mock_emg:=true
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
    mock_emg_arg = DeclareLaunchArgument(
        "mock_emg",
        default_value="false",
        description="Launch mock EMG publisher (cycles through gestures)",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Mia Hand",
    )
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="/prosthesis_ws/tests/emg_volitional/emg_volitional_config.yaml",
        description="Path to volitional controller YAML config",
    )

    mock_emg = LaunchConfiguration("mock_emg")
    serial_port = LaunchConfiguration("serial_port")
    config_path = LaunchConfiguration("config_path")

    # Mia Hand system interface — with RViz and individual controllers
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
            "use_mock_hardware": "false",
            "rviz2_gui": "true",
        }.items(),
    )

    # Volitional controller node
    volitional_node = ExecuteProcess(
        cmd=[
            "python3",
            "/prosthesis_ws/tests/emg_volitional/emg_volitional_node.py",
            "--ros-args",
            "-p",
            ["config_path:=", config_path],
        ],
        output="screen",
    )

    # Mock EMG publisher (cycles through gestures for demo)
    mock_emg_node = GroupAction(
        condition=IfCondition(mock_emg),
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    "/prosthesis_ws/tests/emg_volitional/mock_emg_publisher.py",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        mock_emg_arg,
        serial_port_arg,
        config_path_arg,
        mia_hand_launch,
        volitional_node,
        mock_emg_node,
    ])
