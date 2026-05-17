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
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _condition(expr) -> IfCondition:
    return IfCondition(PythonExpression([
        *expr
    ]))


def generate_launch_description():
    pkg = FindPackageShare("sensor_fusion_bringup")
    use_fallback = LaunchConfiguration("use_marker_odometry_fallback")
    marker_tf_max_age_s = LaunchConfiguration("marker_tf_max_age_s")

    head_openvins_mixed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "head_d435_openvins_phase2.launch.py"])
        ),
        condition=_condition(["'", LaunchConfiguration("rig_mode"), "' == 'mixed_d435_d435i' and '", use_fallback, "' == 'false'"]),
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
        condition=_condition(["'", LaunchConfiguration("rig_mode"), "' == 'dual_d435i' and '", use_fallback, "' == 'false'"]),
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
        condition=_condition(["'", use_fallback, "' == 'false'"]),
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
            "config_file": PathJoinSubstitution([pkg, "config", "markers", "head_aruco_map.yaml"]),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    arm_marker_pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "arm_marker_pose_phase2.launch.py"])
        ),
        launch_arguments={
            "config_file": PathJoinSubstitution([pkg, "config", "markers", "arm_aruco_map.yaml"]),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }.items(),
    )

    head_marker_odom_bridge = Node(
        package="sensor_fusion_bringup",
        executable="marker_pose_odometry_bridge.py",
        name="head_marker_pose_odometry_bridge",
        output="screen",
        condition=_condition(["'", use_fallback, "' == 'true'"]),
        parameters=[{
            "input_pose_topic": "/head/marker_pose/imu_pose",
            "output_odom_topic": "/ov_msckf_head/odomimu",
            "odom_child_frame": "head_imu",
            "tf_child_frame": "d435i_head_link",
            "max_cached_tf_age_s": marker_tf_max_age_s,
        }],
    )

    arm_marker_odom_bridge = Node(
        package="sensor_fusion_bringup",
        executable="marker_pose_odometry_bridge.py",
        name="arm_marker_pose_odometry_bridge",
        output="screen",
        condition=_condition(["'", use_fallback, "' == 'true'"]),
        parameters=[{
            "input_pose_topic": "/arm/marker_pose/imu_pose",
            "output_odom_topic": "/ov_msckf_arm/odomimu",
            "odom_child_frame": "arm_imu",
            "tf_child_frame": "d435i_arm_link",
            "max_cached_tf_age_s": marker_tf_max_age_s,
        }],
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
        DeclareLaunchArgument(
            "use_marker_odometry_fallback",
            default_value="true",
            description="Use ArUco pose as OpenVINS-compatible odom when marker estimator binary is unavailable.",
        ),
        DeclareLaunchArgument(
            "marker_tf_max_age_s",
            default_value="2.0",
            description="Seconds to keep publishing the last marker-derived camera TF after marker tracking drops.",
        ),
        head_openvins_mixed,
        head_openvins_d435i,
        arm_openvins,
        head_marker_pose,
        arm_marker_pose,
        head_marker_odom_bridge,
        arm_marker_odom_bridge,
    ])
