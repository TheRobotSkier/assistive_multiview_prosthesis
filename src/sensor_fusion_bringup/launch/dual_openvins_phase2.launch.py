"""Dual-camera OpenVINS Phase 2 bringup.

rig_mode:
  mixed_d435_d435i  Head D435 + external I2C IMU, arm D435i.
  dual_d435i        Final two-D435i rig.

This launch file is intended to run in the openVINS container with start_camera:=false.
The cameras container (dual_d435i.launch.py) provides camera and IMU topics.

OpenVINS nodes are delayed 5 s each (handled inside the per-camera phase2 launch files).
The marker pose nodes are delayed an additional 2 s to allow OpenVINS to initialise
before observations start flowing (total ~7 s from launch).
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _rig_is(name: str) -> IfCondition:
    return IfCondition(PythonExpression([
        "'", LaunchConfiguration("rig_mode"), "' == '", name, "'"
    ]))


def generate_launch_description():
    pkg = FindPackageShare("sensor_fusion_bringup")

    head_openvins_mixed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "head_d435_openvins_phase2.launch.py"])
        ),
        condition=_rig_is("mixed_d435_d435i"),
        launch_arguments={
            "verbosity": LaunchConfiguration("verbosity"),
            "start_camera": LaunchConfiguration("start_camera"),
            "start_external_imu": LaunchConfiguration("start_external_imu"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    head_openvins_d435i = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "head_d435i_openvins_phase2.launch.py"])
        ),
        condition=_rig_is("dual_d435i"),
        launch_arguments={
            "verbosity": LaunchConfiguration("verbosity"),
            "start_camera": LaunchConfiguration("start_camera"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    arm_openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "arm_d435i_openvins_phase2.launch.py"])
        ),
        launch_arguments={
            "verbosity": LaunchConfiguration("verbosity"),
            "start_camera": LaunchConfiguration("start_camera"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    head_marker_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "head_marker_pose_phase2.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    arm_marker_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "arm_marker_pose_phase2.launch.py"])
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "rig_mode",
            default_value="dual_d435i",
            description="dual_d435i for final rig, mixed_d435_d435i for fallback bench rig.",
        ),
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument(
            "start_camera",
            default_value="false",
            description="Start live RealSense cameras. Leave false when cameras container is running.",
        ),
        DeclareLaunchArgument(
            "start_external_imu",
            default_value="false",
            description="Start head external IMU here. Leave false when mixed cameras container is running.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock (true for rosbag replay).",
        ),
        head_openvins_mixed,
        head_openvins_d435i,
        arm_openvins,
        # Marker pose nodes wait for OpenVINS to be ready (openvins delays 5 s;
        # add 2 s here for a total of ~7 s before marker observations flow).
        TimerAction(period=7.0, actions=[head_marker_pose]),
        TimerAction(period=7.0, actions=[arm_marker_pose]),
    ])
