"""Dual-camera OpenVINS Phase 2 bringup (head + arm D435i).

Launches:
  - Head D435i camera + OpenVINS marker-fusion estimator
  - Arm D435i camera + OpenVINS marker-fusion estimator
  - Head ArUco marker pose node (rate-limited)
  - Arm ArUco marker pose node (rate-limited)

Both cameras start immediately. OpenVINS nodes are delayed 5 s each
(handled inside the per-camera phase2 launch files).
The marker pose nodes are delayed an additional 2 s to allow OpenVINS
to initialise before observations start flowing.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("sensor_fusion_bringup")

    head_openvins = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "head_d435i_openvins_phase2.launch.py"])
        ),
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
        DeclareLaunchArgument("verbosity", default_value="INFO"),
        DeclareLaunchArgument(
            "start_camera",
            default_value="true",
            description="Start live RealSense cameras. Set false for bag replay.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock (true for rosbag replay).",
        ),
        head_openvins,
        arm_openvins,
        # Marker pose nodes wait for OpenVINS to be ready (openvins delays 5 s;
        # add 2 s here for a total of ~7 s before marker observations flow).
        TimerAction(period=7.0, actions=[head_marker_pose]),
        TimerAction(period=7.0, actions=[arm_marker_pose]),
    ])
