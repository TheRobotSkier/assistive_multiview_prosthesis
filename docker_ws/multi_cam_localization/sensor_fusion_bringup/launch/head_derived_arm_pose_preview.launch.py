from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("sensor_fusion_bringup")

    default_head_config = os.path.join(
        pkg_share,
        "config",
        "markers",
        "head_aruco_map.yaml",
    )
    default_arm_marker_extrinsics = os.path.join(
        pkg_share,
        "config",
        "markers",
        "arm_marker_extrinsics.yaml",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "head_marker_config",
            default_value=default_head_config,
            description="Path to the head marker YAML config used for head camera frame and T_cam_imu.",
        ),
        DeclareLaunchArgument(
            "arm_marker_extrinsics",
            default_value=default_arm_marker_extrinsics,
            description="Path to the calibrated arm-mounted marker extrinsic YAML.",
        ),
        DeclareLaunchArgument(
            "dynamic_observation_topic",
            default_value="/head/marker_pose/dynamic_observation",
            description="Dynamic marker observation topic from the head marker node.",
        ),
        DeclareLaunchArgument(
            "head_pose_topic",
            default_value="/ov_msckf/poseimu",
            description="Head OpenVINS PoseWithCovarianceStamped topic in marker_map.",
        ),
        DeclareLaunchArgument(
            "output_prefix",
            default_value="/arm/marker_pose/head_derived",
            description="Output prefix for debug-only head-derived arm preview topics.",
        ),
        DeclareLaunchArgument(
            "marker_id",
            default_value="2",
            description="Dynamic arm-mounted marker ID to consume.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock, typically true during rosbag replay.",
        ),
        DeclareLaunchArgument(
            "require_stable_dynamic_marker",
            default_value="true",
            description="Require the dynamic marker observation stable flag before publishing preview poses.",
        ),
        DeclareLaunchArgument(
            "publish_tf",
            default_value="true",
            description="Publish marker_map to *_head_preview TF frames.",
        ),

        Node(
            package="sensor_fusion_bringup",
            executable="head_derived_arm_pose_preview_node.py",
            name="head_derived_arm_pose_preview_node",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"head_marker_config": LaunchConfiguration("head_marker_config")},
                {"arm_marker_extrinsics": LaunchConfiguration("arm_marker_extrinsics")},
                {"dynamic_observation_topic": LaunchConfiguration("dynamic_observation_topic")},
                {"head_pose_topic": LaunchConfiguration("head_pose_topic")},
                {"output_prefix": LaunchConfiguration("output_prefix")},
                {"marker_id": ParameterValue(LaunchConfiguration("marker_id"), value_type=int)},
                {
                    "require_stable_dynamic_marker": ParameterValue(
                        LaunchConfiguration("require_stable_dynamic_marker"),
                        value_type=bool,
                    )
                },
                {"publish_tf": ParameterValue(LaunchConfiguration("publish_tf"), value_type=bool)},
            ],
        ),
    ])
