"""Launch file for EMG-driven velocity-based force grasp test.

Launches:
  - Mia Hand ros2_control (with group position + velocity controllers)
  - Wrist Dynamixel driver
  - Mia safety node
  - EMG grasp test node
  - Live EMG classifier (when emg:=true)

Usage:
  ros2 launch prosthesis_launch emg_grasp_test.launch.py
  ros2 launch prosthesis_launch emg_grasp_test.launch.py emg:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    emg_arg = DeclareLaunchArgument(
        "emg",
        default_value="false",
        description="Launch live EMG classifier + ROS bridge",
    )
    mock_emg_arg = DeclareLaunchArgument(
        "mock_emg",
        default_value="false",
        description="Launch mock EMG publisher (gesture=1, duration=5s)",
    )
    launch_mia_hand_arg = DeclareLaunchArgument(
        "launch_mia_hand",
        default_value="true",
        description="Launch the local Mia Hand ros2_control stack",
    )
    launch_wrist_driver_arg = DeclareLaunchArgument(
        "launch_wrist_driver",
        default_value="true",
        description="Launch the local wrist driver",
    )
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for Mia Hand",
    )
    wrist_serial_port_arg = DeclareLaunchArgument(
        "wrist_serial_port",
        default_value="/dev/ttyUSB1",
        description="Serial port for wrist Dynamixel servo",
    )
    config_path_arg = DeclareLaunchArgument(
        "config_path",
        default_value="/prosthesis_ws/tests/emg_grasp/emg_grasp_test.yaml",
        description="Path to EMG grasp test YAML config",
    )
    model_dir_arg = DeclareLaunchArgument(
        "model_dir",
        default_value="/prosthesis_ws/models",
        description="Directory containing trained EMG classifier model files",
    )
    emg_config_arg = DeclareLaunchArgument(
        "emg_config",
        default_value="",
        description="Path to emg_experiment_config.yaml for experimental modes",
    )
    use_mock_hardware_arg = DeclareLaunchArgument(
        "use_mock_hardware",
        default_value="false",
        description="Launch Mia Hand ros2_control using mock hardware",
    )

    emg = LaunchConfiguration("emg")
    mock_emg = LaunchConfiguration("mock_emg")
    launch_mia_hand = LaunchConfiguration("launch_mia_hand")
    launch_wrist_driver = LaunchConfiguration("launch_wrist_driver")
    serial_port = LaunchConfiguration("serial_port")
    wrist_serial_port = LaunchConfiguration("wrist_serial_port")
    config_path = LaunchConfiguration("config_path")
    model_dir = LaunchConfiguration("model_dir")
    emg_config = LaunchConfiguration("emg_config")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Mia Hand system interface — spawns group pos+vel controllers so the EMG
    # grasp test node can command all fingers with a single Float64MultiArray.
    mia_hand_launch = GroupAction(
        condition=IfCondition(launch_mia_hand),
        actions=[
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
                    "serial_port": serial_port,
                    "controller": "",
                    "use_group_controllers": "true",
                    "use_mock_hardware": use_mock_hardware,
                    "rviz2_gui": "false",
                }.items(),
            )
        ],
    )

    # Wrist Dynamixel driver
    wrist_driver_node = Node(
        condition=IfCondition(launch_wrist_driver),
        package="wrist_driver",
        executable="wrist_driver_node",
        name="wrist_driver",
        parameters=[{"port": wrist_serial_port}],
        output="screen",
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

    # Live EMG classifier + ROS bridge (publishes /emg/gesture_label, /emg/confidence, etc.)
    # Requires trained model files at model_dir (default: /prosthesis_ws/models).
    # The wrapper retries until the MindRove armband is reachable and yields to
    # an already-running external EMG publisher if one is detected.
    emg_bridge_node = GroupAction(
        condition=IfCondition(emg),
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    "/prosthesis_ws/src/emg_bridge/scripts/run_classifier_retry.py",
                    "--model-dir",
                    model_dir,
                    "--config",
                    emg_config,
                ],
                output="screen",
            ),
        ],
    )

    # Mock EMG publisher (for non-hardware testing)
    mock_emg_node = GroupAction(
        condition=IfCondition(mock_emg),
        actions=[
            ExecuteProcess(
                cmd=[
                    "python3",
                    "/prosthesis_ws/tests/emg_grasp/mock_emg_publisher.py",
                    "--gesture",
                    "1",
                    "--confidence",
                    "0.95",
                    "--duration",
                    "5.0",
                ],
                output="screen",
            ),
        ],
    )

    return LaunchDescription(
        [
            emg_arg,
            mock_emg_arg,
            launch_mia_hand_arg,
            launch_wrist_driver_arg,
            model_dir_arg,
            emg_config_arg,
            serial_port_arg,
            wrist_serial_port_arg,
            config_path_arg,
            use_mock_hardware_arg,
            mia_hand_launch,
            wrist_driver_node,
            emg_grasp_node,
            emg_bridge_node,
            mock_emg_node,
        ]
    )
