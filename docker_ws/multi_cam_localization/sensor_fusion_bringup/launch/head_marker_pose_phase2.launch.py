from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("sensor_fusion_bringup")

    default_config = os.path.join(
        pkg_share,
        "config",
        "markers",
        "head_aruco_map.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=default_config,
            description="Path to marker map YAML (Python node only)",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock, typically true during rosbag replay.",
        ),
        DeclareLaunchArgument(
            "marker_detection_rate_hz",
            default_value="15.0",
            description="Maximum ArUco detection rate. Set 0.0 to process every image frame.",
        ),
        DeclareLaunchArgument(
            "use_cpp_marker_pose",
            default_value="false",
            description="Use C++ ArUco marker pose node instead of Python",
        ),

        # ── Python node (default) ──────────────────────────────────────
        Node(
            package="sensor_fusion_bringup",
            executable="aruco_marker_pose_node.py",
            name="aruco_marker_pose_node_phase2",
            output="screen",
            condition=UnlessCondition(LaunchConfiguration("use_cpp_marker_pose")),
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"config_file": LaunchConfiguration("config_file")},
                {"correction_enabled_override": "false"},
                {"marker_detection_rate_hz": ParameterValue(LaunchConfiguration("marker_detection_rate_hz"), value_type=float)},
            ],
        ),

        # ── C++ node (opt-in) ──────────────────────────────────────────
        Node(
            package="sensor_fusion_bringup",
            executable="aruco_marker_pose_cpp_node",
            name="aruco_marker_pose_cpp_node_phase2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_cpp_marker_pose")),
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"image_topic": "/head/d435i_head/color/image_raw"},
                {"camera_info_topic": "/head/d435i_head/color/camera_info"},
                {"output_topic_prefix": "/head/marker_pose"},
                {"output_topic": "/head/marker_pose/observation"},
                {"target_frame": "head_imu"},
                {"marker_map_frame": "marker_map"},
                {"camera_frame": "head_d435i_head_color_optical_frame"},
                {"marker_id": 0},
                {"marker_frame": "marker_0"},
                {"marker_size_m": 0.100},
                {"marker_dictionary": "DICT_6X6_1000"},
                {"min_marker_area_px2": 800.0},
                {"max_reprojection_error_px": 3.0},
                {"max_marker_distance_m": 2.0},
                {"stable_frames_required": 8},
                {"marker_detection_rate_hz": ParameterValue(LaunchConfiguration("marker_detection_rate_hz"), value_type=float)},
                # T_cam_imu and T_map_marker default to identity; override
                # with the Kalibr head calibration and marker map values.
            ],
        ),
    ])
