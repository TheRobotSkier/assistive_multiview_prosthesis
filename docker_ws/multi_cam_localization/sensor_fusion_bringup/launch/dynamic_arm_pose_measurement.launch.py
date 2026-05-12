from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("sensor_fusion_bringup")

    default_head_config = os.path.join(pkg_share, "config", "markers", "head_aruco_map.yaml")
    default_arm_config = os.path.join(pkg_share, "config", "markers", "arm_aruco_map.yaml")
    default_arm_marker_extrinsics = os.path.join(pkg_share, "config", "markers", "arm_marker_extrinsics.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "head_marker_config",
            default_value=default_head_config,
            description="Path to the head marker YAML config used for head camera frame and T_cam_imu.",
        ),
        DeclareLaunchArgument(
            "arm_marker_config",
            default_value=default_arm_config,
            description="Path to the arm marker YAML config used for arm camera frame and T_cam_imu.",
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
            default_value="/ov_msckf/odomimu",
            description="Head OpenVINS pose topic in marker_map.",
        ),
        DeclareLaunchArgument(
            "head_pose_message_type",
            default_value="odometry",
            description="Message type on head_pose_topic: odometry or pose_with_covariance_stamped.",
        ),
        DeclareLaunchArgument(
            "dynamic_arm_pose_observation_topic",
            default_value="/arm/marker_pose/dynamic_arm_pose_observation",
            description="OpenVINS-facing dynamic arm pose observation topic.",
        ),
        DeclareLaunchArgument(
            "dynamic_arm_measurement_status_topic",
            default_value="/arm/marker_pose/dynamic_arm_measurement/status",
            description="Status topic for the dynamic arm pose measurement producer.",
        ),
        DeclareLaunchArgument(
            "publish_dynamic_arm_pose_observation",
            default_value="false",
            description="Publish OpenVINS-facing dynamic arm pose observations. False is measurement-only/status mode.",
        ),
        DeclareLaunchArgument("marker_id", default_value="2", description="Dynamic arm-mounted marker ID to consume."),
        DeclareLaunchArgument("target_frame", default_value="arm_imu", description="Arm OpenVINS target frame."),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock, typically true during rosbag replay.",
        ),
        DeclareLaunchArgument(
            "require_stable_dynamic_marker",
            default_value="true",
            description="Require the dynamic marker observation stable flag before publishing measurements.",
        ),
        DeclareLaunchArgument(
            "max_head_pose_dt_s",
            default_value="0.05",
            description="Maximum absolute timestamp difference between ID2 observation and already-buffered head pose.",
        ),
        Node(
            package="sensor_fusion_bringup",
            executable="dynamic_arm_pose_measurement_node.py",
            name="dynamic_arm_pose_measurement_node",
            output="screen",
            parameters=[
                {"use_sim_time": LaunchConfiguration("use_sim_time")},
                {"head_marker_config": LaunchConfiguration("head_marker_config")},
                {"arm_marker_config": LaunchConfiguration("arm_marker_config")},
                {"arm_marker_extrinsics": LaunchConfiguration("arm_marker_extrinsics")},
                {"dynamic_observation_topic": LaunchConfiguration("dynamic_observation_topic")},
                {"head_pose_topic": LaunchConfiguration("head_pose_topic")},
                {"head_pose_message_type": LaunchConfiguration("head_pose_message_type")},
                {"dynamic_arm_pose_observation_topic": LaunchConfiguration("dynamic_arm_pose_observation_topic")},
                {"dynamic_arm_measurement_status_topic": LaunchConfiguration("dynamic_arm_measurement_status_topic")},
                {
                    "publish_dynamic_arm_pose_observation": ParameterValue(
                        LaunchConfiguration("publish_dynamic_arm_pose_observation"),
                        value_type=bool,
                    )
                },
                {"marker_id": ParameterValue(LaunchConfiguration("marker_id"), value_type=int)},
                {"target_frame": LaunchConfiguration("target_frame")},
                {
                    "require_stable_dynamic_marker": ParameterValue(
                        LaunchConfiguration("require_stable_dynamic_marker"),
                        value_type=bool,
                    )
                },
                {"max_head_pose_dt_s": ParameterValue(LaunchConfiguration("max_head_pose_dt_s"), value_type=float)},
            ],
        ),
    ])
