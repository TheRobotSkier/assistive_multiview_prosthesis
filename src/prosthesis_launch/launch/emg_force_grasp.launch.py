"""Launch file for EMG-driven force-aware grasp test.

Extends emg_grasp_test.launch.py with the force controller pipeline:

Launches:
  - Mia Hand ros2_control (with group + per-finger controllers)
  - Force bridge node (joint effort → ForceData)
  - Force controller node (PI force regulation)
  - Wrist Dynamixel driver
  - EMG-driven grasp test node (force-aware variant)
  - Live EMG classifier (when emg:=true)

Usage:
  ros2 launch prosthesis_launch emg_force_grasp.launch.py
  ros2 launch prosthesis_launch emg_force_grasp.launch.py emg:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
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
        default_value="/prosthesis_ws/tests/emg_grasp/emg_grasp_test_force.yaml",
        description="Path to EMG force grasp test YAML config",
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

    # Mia Hand system interface — spawns group controllers first, then
    # per-finger position controllers (inactive) switched on by emg_grasp_node.
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
            ),
            # Spawn per-finger position controllers as inactive (delayed so
            # group controllers are loaded first).
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="thumb_pos_spawner",
                        arguments=[
                            "thumb_pos_ff_controller",
                            "-c", "/controller_manager",
                            "--inactive",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="index_pos_spawner",
                        arguments=[
                            "index_pos_ff_controller",
                            "-c", "/controller_manager",
                            "--inactive",
                        ],
                        output="screen",
                    ),
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        name="mrl_pos_spawner",
                        arguments=[
                            "mrl_pos_ff_controller",
                            "-c", "/controller_manager",
                            "--inactive",
                        ],
                        output="screen",
                    ),
                ],
            ),
        ],
    )

    # Force bridge: joint_states effort → ForceData
    force_bridge_node = Node(
        package="force_controller",
        executable="force_bridge_node",
        name="force_bridge",
        parameters=[{
            "joint_states_topic": "/joint_states",
            "force_data_topic": "data_streams/fingers/forces/data",
            "force_switch_service": "data_streams/fingers/forces/switch",
            "publish_rate_hz": 100.0,
        }],
        output="screen",
    )

    # Force controller: PI force regulation
    force_controller_node = Node(
        package="force_controller",
        executable="force_controller_node",
        name="force_controller",
        parameters=[{
            "force_data_topic": "data_streams/fingers/forces/data",
            "force_stream_switch_service": "data_streams/fingers/forces/switch",
            "pipeline_state_topic": "/pipeline/state",
            "joint_states_topic": "/joint_states",
            "thumb_cmd_topic": "/thumb_pos_ff_controller/commands",
            "index_cmd_topic": "/index_pos_ff_controller/commands",
            "mrl_cmd_topic": "/mrl_pos_ff_controller/commands",
            "update_rate_hz": 10.0,
            "target_force_min": 50.0,
            "target_force_max": 200.0,
            "kp": 0.01,
            "ki": 0.001,
        }],
        output="screen",
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

    # EMG grasp test node (force-aware variant)
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

    # Live EMG classifier + ROS bridge
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
            force_bridge_node,
            force_controller_node,
            wrist_driver_node,
            emg_grasp_node,
            emg_bridge_node,
            mock_emg_node,
        ]
    )
