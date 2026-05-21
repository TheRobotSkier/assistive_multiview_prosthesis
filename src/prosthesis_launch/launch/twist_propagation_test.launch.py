"""Twist Propagation Test Launch — Jetson dual-camera fusion + pipeline nodes.

Brings up everything needed to test twist propagation with dual cameras
streaming from the Jetson.  The cameras + OpenVINS run on the Jetson; this
file only starts the local pipeline nodes.

Both camera point clouds are fused into a single cloud in the marker_map
frame by the pointcloud_fusion_node, then filtered (distance, hand removal,
voxel downsampling) before being consumed by segmentation and twist
propagation.

The arm camera's OpenVINS odometry (/ov_msckf_arm/odomimu) is relayed to
/hand_pose so that moving the camera in real-time drives the twist estimation.

Nodes started:
  1. Pointcloud fusion       — TF-transforms both clouds to marker_map, merges, filters
  2. Odom-to-pose relay      — converts OpenVINS odom to /hand_pose + /hand_twist
  3. Segmentation bridge     — HTTP inference client (subscribes directly to /fused_pointcloud)
  4. Grasp preshaping         — C++/Rust FFI bridge (called after segmentation)
  5. Twist propagation        — active, listening to /fused_pointcloud + odom
  6. RViz                     — twist_propagation.rviz (fixed frame: marker_map)

NOT started (must be provided externally):
  - Camera + OpenVINS nodes   — running on Jetson

Usage:
  ros2 launch prosthesis_launch twist_propagation_test.launch.py

  # Without RViz:
  ros2 launch prosthesis_launch twist_propagation_test.launch.py rviz:=false
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_twist_propagation_params():
    """Load the canonical twist_propagation parameters from the package config."""
    config_path = os.path.join(
        get_package_share_directory("twist_propagation"),
        "config",
        "twist_propagation.yaml",
    )
    if os.path.exists(config_path):
        with open(config_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        return dict(cfg.get("twist_propagation", {}).get("ros__parameters", {}))
    return {}


def _launch_setup(context, *args, **kwargs):
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    active = LaunchConfiguration("active").perform(context)
    inference_url = LaunchConfiguration("inference_url").perform(context)
    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1", "yes")
    use_marker_map = LaunchConfiguration("use_marker_map_clouds").perform(context).lower() in ("true", "1", "yes")
    cam1_topic = LaunchConfiguration("cam1_topic").perform(context)
    cam2_topic = LaunchConfiguration("cam2_topic").perform(context)
    target_frame = LaunchConfiguration("target_frame").perform(context)
    arm_frame = LaunchConfiguration("arm_frame").perform(context)

    if use_marker_map:
        cam1_topic = "/head/d435i_head/points_marker_map"
        cam2_topic = "/arm/d435i_arm/points_marker_map"
        target_frame = "marker_map"

    nodes = []

    # ── 1. Pointcloud fusion ─────────────────────────────────────────────
    # Subscribes to both Jetson camera clouds, transforms them to marker_map
    # frame via TF2 (provided by OpenVINS on the Jetson), merges, applies
    # distance filtering, hand/arm bbox removal, and voxel downsampling.
    nodes.append(
        Node(
            package="pointcloud_fusion",
            executable="pointcloud_fusion_node",
            name="pointcloud_fusion",
            parameters=[{
                "target_frame": target_frame,
                "cam1_topic": cam1_topic,
                "cam2_topic": cam2_topic,
                "arm_frame": arm_frame,
                "max_distance": 2.0,
                "voxel_size": 0.005,
                "bbox_min": [-0.30, -0.10, -0.10],
                "bbox_max": [0.22, 0.10, 0.12],
                "enable_downsampling": True,
                "enable_distance_filter": True,
                "enable_hand_removal": True,
            }],
            output="screen",
        )
    )

    # ── 2. Odom-to-pose relay ────────────────────────────────────────────
    # Converts the OpenVINS odometry stream into /hand_pose (PoseStamped),
    # /hand_twist (TwistStamped), and /hand_odom (full Odometry for covariance).
    nodes.append(
        Node(
            package="camera",
            executable="odom_to_pose_relay",
            name="odom_to_pose_relay",
            parameters=[{
                "odom_topic": odom_topic,
                "pose_topic": "/hand_pose",
                "twist_topic": "/hand_twist",
                "odom_out": "/hand_odom",
            }],
            output="screen",
            arguments=["--ros-args", "--log-level", "warn"],
        )
    )

    # ── 3. Segmentation bridge ───────────────────────────────────────────
    # Subscribes directly to /fused_pointcloud via remapping.
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="segmentation_ros2_node",
            name="segmentation_bridge",
            remappings={("/segmentation/input_cloud", "/fused_pointcloud")},
            parameters=[{"inference_url": inference_url}],
            output="screen",
        )
    )

    # ── 4. Grasp preshaping service ──────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="preshaping_service_bridge_node",
            name="preshaping_service",
            output="screen",
        )
    )

    # ── 5. Twist propagation ─────────────────────────────────────────────
    # Load full parameter set from the package config, overlay overrides.
    twist_params = _load_twist_propagation_params()
    twist_params["active"] = active == "true"
    twist_params["input_cloud_topic"] = "/fused_pointcloud"
    twist_params["odom_topic"] = "/hand_odom"

    nodes.append(
        Node(
            package="twist_propagation",
            executable="twist_propagation_node",
            name="twist_propagation",
            parameters=[twist_params],
            output="screen",
        )
    )

    # ── 6. RViz ──────────────────────────────────────────────────────────
    if rviz_enabled:
        # The rviz/ directory lives at /prosthesis_ws/rviz/ inside the container.
        # Use absolute path since rviz configs aren't part of any ROS package.
        # get_package_share_directory returns .../install/prosthesis_launch/share/prosthesis_launch
        # so we need 4 dirname calls to reach /prosthesis_ws/.
        share_dir = get_package_share_directory("prosthesis_launch")
        workspace_dir = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(share_dir))))
        rviz_config = os.path.join(workspace_dir, "rviz", "twist_propagation.rviz")
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/ov_msckf_arm/odomimu",
            description="OpenVINS odometry topic for hand pose estimation.",
        ),
        DeclareLaunchArgument(
            "active",
            default_value="true",
            description="Start twist propagation in active mode.",
        ),
        DeclareLaunchArgument(
            "inference_url",
            default_value="http://127.0.0.1:5678",
            description="Segmentation inference server URL.",
        ),
        DeclareLaunchArgument(
            "use_marker_map_clouds",
            default_value="false",
            description="Use pre-transformed marker_map pointclouds instead of raw depth/color/points.",
        ),
        DeclareLaunchArgument(
            "cam1_topic",
            default_value="/head/d435i_head/depth/color/points",
            description="Camera 1 pointcloud topic.",
        ),
        DeclareLaunchArgument(
            "cam2_topic",
            default_value="/arm/d435i_arm/depth/color/points",
            description="Camera 2 pointcloud topic.",
        ),
        DeclareLaunchArgument(
            "target_frame",
            default_value="marker_map",
            description="Target frame for fused pointcloud.",
        ),
        DeclareLaunchArgument(
            "arm_frame",
            default_value="arm_d435i_arm_depth_frame",
            description="Arm frame for distance filtering and bbox removal.",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz with twist_propagation.rviz.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
