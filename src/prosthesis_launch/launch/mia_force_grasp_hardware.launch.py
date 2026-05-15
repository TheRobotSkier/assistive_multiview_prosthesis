"""MIA force-aware grasping hardware launch.

Launches the complete ros2_control-based hardware stack:

  1. ros2_control (controller_manager + robot_state_publisher)
  2. Per-finger position/velocity controllers (force-aware path)
  3. Wrist Dynamixel driver
  4. Hand pose publisher (camera package)
  5. Preshaping service bridge
  6. Grasp proximity controller
  7. Pipeline manager (state machine)

Does NOT launch mia_hand_driver_node (standalone driver — conflicts with ros2_control)
or force_controller_node.

Usage:
  ros2 launch prosthesis_launch mia_force_grasp_hardware.launch.py
  ros2 launch prosthesis_launch mia_force_grasp_hardware.launch.py \
    serial_port:=/dev/ttyUSB0 wrist_port:=/dev/ttyUSB1 \
    config_file:=/path/to/config.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/mia_hand",
        description="Serial port for Mia Hand (persistent symlink)",
    )

    wrist_port_arg = DeclareLaunchArgument(
        "wrist_port",
        default_value="/dev/wrist_motor",
        description="Serial port for wrist Dynamixel motor",
    )

    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description="Path to prosthesis_config.yaml (empty = package default)",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz (off by default for hardware-only)",
    )

    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="false",
        description="Launch RealSense camera pipeline",
    )

    segmentation_arg = DeclareLaunchArgument(
        "segmentation",
        default_value="false",
        description="Launch segmentation ROS bridge",
    )

    emg_arg = DeclareLaunchArgument(
        "emg",
        default_value="false",
        description="Launch EMG bridge (MindRove)",
    )

    mia_hand_arg = DeclareLaunchArgument(
        "mia_hand",
        default_value="true",
        description="Launch ros2_control stack for Mia Hand",
    )

    wrist_arg = DeclareLaunchArgument(
        "wrist",
        default_value="true",
        description="Launch wrist Dynamixel driver",
    )

    # ── ros2_control: controller_manager + robot_state_publisher + controllers ─
    mia_hand_launch_path = os.path.join(
        get_package_share_directory("mia_hand_ros2_control"),
        "launch",
        "mia_hand_system_interface_launch.py",
    )

    mia_hand_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mia_hand_launch_path),
        launch_arguments={
            "serial_port": LaunchConfiguration("serial_port"),
            "rviz2_gui": LaunchConfiguration("rviz"),
            "use_mock_hardware": "false",
            "controller": "",
        }.items(),
    )

    # ── Wrist Dynamixel driver ────────────────────────────────────────────
    wrist_driver = Node(
        package="wrist_driver",
        executable="wrist_driver_node",
        name="wrist_driver",
        parameters=[{"port": LaunchConfiguration("wrist_port")}],
        output="screen",
    )

    # ── Hand pose publisher ───────────────────────────────────────────────
    hand_pose = Node(
        package="camera",
        executable="hand_pose_publisher",
        name="hand_pose_publisher",
        output="screen",
    )

    # ── Preshaping service bridge (C++/Rust FFI) ─────────────────────────
    preshaping = Node(
        package="grasp_preshaping",
        executable="preshaping_service_bridge_node",
        name="preshaping_service",
        output="screen",
    )

    # ── Grasp proximity controller ────────────────────────────────────────
    proximity = Node(
        package="grasp_preshaping",
        executable="grasp_proximity_controller_node.py",
        name="proximity_controller",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
        output="screen",
    )

    # ── Pipeline manager (state machine) ──────────────────────────────────
    pipeline_manager = Node(
        package="pipeline_manager",
        executable="pipeline_manager_node",
        name="pipeline_manager",
        parameters=[{"config_file": LaunchConfiguration("config_file")}],
        output="screen",
    )

    return LaunchDescription([
        serial_port_arg,
        wrist_port_arg,
        config_file_arg,
        rviz_arg,
        camera_arg,
        segmentation_arg,
        emg_arg,
        mia_hand_arg,
        wrist_arg,
        mia_hand_ros2_control,
        wrist_driver,
        hand_pose,
        preshaping,
        proximity,
        pipeline_manager,
    ])
