from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _pointcloud_transform_node(camera: str) -> Node:
    return Node(
        package="sensor_fusion_bringup",
        executable="pointcloud_to_frame_node",
        name=f"{camera}_d435i_raw_points_to_marker_map",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"input_topic": LaunchConfiguration(f"{camera}_input_topic")},
            {"output_topic": LaunchConfiguration(f"{camera}_output_topic")},
            {"target_frame": LaunchConfiguration("target_frame")},
            {"camera_pose_frame": LaunchConfiguration(f"{camera}_camera_pose_frame")},
            {"camera_color_optical_frame": LaunchConfiguration(f"{camera}_camera_color_optical_frame")},
            {"marker_map_locked_topic": LaunchConfiguration(f"{camera}_marker_map_locked_topic")},
            {
                "require_marker_map_locked": ParameterValue(
                    LaunchConfiguration("pointcloud_require_marker_map_locked"),
                    value_type=bool,
                )
            },
            {"max_rate_hz": ParameterValue(LaunchConfiguration("pointcloud_max_rate_hz"), value_type=float)},
            {"voxel_leaf_m": ParameterValue(LaunchConfiguration("pointcloud_voxel_leaf_m"), value_type=float)},
            {"max_source_range_m": ParameterValue(LaunchConfiguration("pointcloud_max_range_m"), value_type=float)},
            {"transform_timeout_s": ParameterValue(LaunchConfiguration("pointcloud_transform_timeout_s"), value_type=float)},
            {"max_tf_age_s": ParameterValue(LaunchConfiguration("pointcloud_max_tf_age_s"), value_type=float)},
            {"use_latest_tf": ParameterValue(LaunchConfiguration("pointcloud_use_latest_tf"), value_type=bool)},
        ],
    )


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("target_frame", default_value="marker_map"),
            DeclareLaunchArgument(
                "head_input_topic",
                default_value="/head/d435i_head/depth/color/points",
                description="Canonical raw head D435i color pointcloud topic from the Jetson.",
            ),
            DeclareLaunchArgument(
                "arm_input_topic",
                default_value="/arm/d435i_arm/depth/color/points",
                description="Canonical raw arm D435i color pointcloud topic from the Jetson.",
            ),
            DeclareLaunchArgument(
                "head_output_topic",
                default_value="/head/d435i_head/points_marker_map",
                description="Head cloud transformed into marker_map on this machine.",
            ),
            DeclareLaunchArgument(
                "arm_output_topic",
                default_value="/arm/d435i_arm/points_marker_map",
                description="Arm cloud transformed into marker_map on this machine.",
            ),
            DeclareLaunchArgument("head_camera_pose_frame", default_value="head_cam0_corrected"),
            DeclareLaunchArgument("arm_camera_pose_frame", default_value="arm_cam0_corrected"),
            DeclareLaunchArgument("head_camera_color_optical_frame", default_value="head_d435i_head_color_optical_frame"),
            DeclareLaunchArgument("arm_camera_color_optical_frame", default_value="arm_d435i_arm_color_optical_frame"),
            DeclareLaunchArgument("head_marker_map_locked_topic", default_value="/ov_msckf/marker_map_locked"),
            DeclareLaunchArgument("arm_marker_map_locked_topic", default_value="/ov_msckf_arm/marker_map_locked"),
            DeclareLaunchArgument(
                "pointcloud_require_marker_map_locked",
                default_value="false",
                description="Drop clouds until OpenVINS reports marker-map lock. False gates only on TF availability.",
            ),
            DeclareLaunchArgument(
                "pointcloud_max_rate_hz",
                default_value="0.0",
                description="Maximum transformed output rate. 0 disables rate limiting for x86/offboard processing.",
            ),
            DeclareLaunchArgument(
                "pointcloud_voxel_leaf_m",
                default_value="0.0",
                description="Voxel leaf size. 0 disables voxel downsampling for full-density x86/offboard processing.",
            ),
            DeclareLaunchArgument(
                "pointcloud_max_range_m",
                default_value="0.0",
                description="Source-frame radial range filter. 0 preserves all finite input points.",
            ),
            DeclareLaunchArgument("pointcloud_transform_timeout_s", default_value="0.05"),
            DeclareLaunchArgument("pointcloud_max_tf_age_s", default_value="0.50"),
            DeclareLaunchArgument(
                "pointcloud_use_latest_tf",
                default_value="true",
                description="Use latest available TF. Helpful when raw clouds and TF arrive over DDS from another machine.",
            ),
            _pointcloud_transform_node("head"),
            _pointcloud_transform_node("arm"),
        ]
    )
