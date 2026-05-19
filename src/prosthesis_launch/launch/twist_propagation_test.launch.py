"""Twist Propagation Test Launch — Jetson arm camera + pipeline nodes.

Brings up everything needed to test twist propagation with a live camera
streaming from the Jetson.  The camera itself runs on the Jetson; this file
only starts the local pipeline nodes.

The arm camera's OpenVINS odometry (/ov_msckf_arm/odomimu) is relayed to
/hand_pose so that moving the camera in real-time drives the twist estimation.

Nodes started:
  1. Odom-to-pose relay     — converts OpenVINS odom to /hand_pose + /hand_twist
  2. Segmentation bridge    — HTTP inference client (input remapped to camera topic)
  3. Grasp preshaping        — C++/Rust FFI bridge (called after segmentation)
  4. Twist propagation       — active, listening to the camera pointcloud + odom
  5. RViz                    — twist_propagation.rviz (fixed frame: marker_map)

NOT started (must be provided externally):
  - Camera + OpenVINS nodes  — running on Jetson

Usage:
  ros2 launch prosthesis_launch twist_propagation_test.launch.py

  # With a different cloud topic:
  ros2 launch prosthesis_launch twist_propagation_test.launch.py \
      input_cloud_topic:=/head/d435i_head/depth/color/points

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
    input_cloud_topic = LaunchConfiguration("input_cloud_topic").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    active = LaunchConfiguration("active").perform(context)
    inference_url = LaunchConfiguration("inference_url").perform(context)
    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower() in ("true", "1", "yes")

    nodes = []

    # ── 1. Odom-to-pose relay ────────────────────────────────────────────
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

    # ── 2. Segmentation bridge ────────────────────────────────────────────
    # Input cloud remapped from the default /segmentation/input_cloud to the
    # actual camera topic coming from the Jetson.
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="segmentation_ros2_node",
            name="segmentation_bridge",
            remappings={("/segmentation/input_cloud", input_cloud_topic)},
            parameters=[{"inference_url": inference_url}],
            output="screen",
        )
    )

    # ── 3. Grasp preshaping service ──────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="preshaping_service_bridge_node",
            name="preshaping_service",
            output="screen",
        )
    )

    # ── 4. Twist propagation ─────────────────────────────────────────────
    # Load full parameter set from the package config, overlay overrides.
    twist_params = _load_twist_propagation_params()
    twist_params["active"] = active == "true"
    twist_params["input_cloud_topic"] = input_cloud_topic
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

    # ── 5. RViz ──────────────────────────────────────────────────────────
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
            "input_cloud_topic",
            default_value="/arm/d435i_arm/points_marker_map",
            description="Pointcloud topic from the Jetson camera.",
        ),
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
            "rviz",
            default_value="true",
            description="Launch RViz with twist_propagation.rviz.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
