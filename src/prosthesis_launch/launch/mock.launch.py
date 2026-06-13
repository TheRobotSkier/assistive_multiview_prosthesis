"""Mock pipeline launch - all nodes with simulated data.

Launches the prosthesis pipeline without hardware:
  - No Mia Hand serial connection
  - No MindRove EMG band
  - No RealSense camera
  - Mock publishers simulate sensor data (clouds, odometry, poses)

Use this for development, testing, and debugging the pipeline logic.

Usage:
  ros2 launch mock.launch.py
  ros2 launch mock.launch.py config_file:=/path/to/config.yaml
  ros2 launch mock.launch.py use_tsdf_fusion:=true
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


# Default config path: try workspace-root config/ first, then relative to launch file
_workspace_root = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."
)
DEFAULT_CONFIG = os.path.join(_workspace_root, "config", "prosthesis_config.yaml")
if not os.path.isfile(DEFAULT_CONFIG):
    # When running from install space, the relative path may not resolve.
    # Fall back to the well-known Docker workspace path.
    DEFAULT_CONFIG = "/prosthesis_ws/config/prosthesis_config.yaml"


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz (set false for headless testing)",
    )

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=DEFAULT_CONFIG,
        description="Path to prosthesis_config.yaml",
    )

    tsdf_arg = DeclareLaunchArgument(
        "use_tsdf_fusion",
        default_value="false",
        description="Use V6 TSDF fusion pipeline instead of the legacy "
                    "segmentation bridge.",
    )

    config = LaunchConfiguration("config_file")
    use_tsdf = LaunchConfiguration("use_tsdf_fusion")

    # ── Mock data publishers ──────────────────────────────────────────────
    # Mock EMG gesture publisher (cycles through gestures)
    mock_emg = Node(
        package="pipeline_manager",
        executable="pipeline_manager_node",
        name="pipeline_manager",
        parameters=[
            config,
            {"use_mock_emg": False},  # test scripts publish synthetic EMG
        ],
        output="screen",
    )

    # Mock point cloud publisher (generates synthetic object cloud)
    mock_cloud = Node(
        package="pipeline_manager",
        executable="mock_cloud_publisher",
        name="mock_cloud_publisher",
        output="screen",
    )

    # Mock odometry + GTSAM pose publisher (for gtsam_tracker / keyframe_buffer)
    mock_odom = Node(
        package="pipeline_manager",
        executable="mock_odom_publisher",
        name="mock_odom_publisher",
        parameters=[{
            "publish_hz": 15.0,
            "publish_gtsam_poses": True,
        }],
        output="screen",
    )

    # ── V6 perception nodes ───────────────────────────────────────────────
    # GTSAM trajectory tracker (factor graph smoother)
    gtsam_tracker = Node(
        package="gtsam_tracker",
        executable="gtsam_tracker_node",
        name="gtsam_tracker",
        parameters=[config],
        output="screen",
    )

    # Keyframe buffer (spatial-gated storage for TSDF fusion)
    keyframe_buffer = Node(
        package="keyframe_buffer",
        executable="keyframe_buffer_node",
        name="keyframe_buffer",
        parameters=[config],
        output="screen",
    )

    # Cross-camera SIFT feature alignment
    cross_camera_features = Node(
        package="cross_camera_features",
        executable="sift_feature_node",
        name="cross_camera_features",
        parameters=[config],
        output="screen",
    )

    # TSDF fusion node (replaces segmentation_bridge when enabled)
    tsdf_fusion = Node(
        package="tsdf_fusion",
        executable="tsdf_fusion_node",
        name="tsdf_fusion",
        parameters=[config],
        output="screen",
        condition=IfCondition(use_tsdf),
    )

    # ── Pipeline nodes ────────────────────────────────────────────────────
    # Segmentation ROS bridge (only when TSDF fusion is disabled)
    segmentation_bridge = Node(
        package="segmentation_bridge",
        executable="segmentation_ros2_node",
        name="segmentation_bridge",
        parameters=[{
            "inference_url": "http://127.0.0.1:5678",
        }],
        output="screen",
        condition=IfCondition(
            PythonExpression(["not ", use_tsdf])),
    )

    # Grasp Preshaping Service
    preshaping_service = Node(
        package="grasp_preshaping",
        executable="preshaping_service_bridge_node",
        name="preshaping_service",
        output="screen",
    )

    # Twist Propagation Target Selector
    twist_propagation = Node(
        package="twist_propagation",
        executable="twist_propagation_node",
        name="twist_propagation",
        parameters=[config],
        output="screen",
    )

    # Grasp Proximity Controller
    proximity_controller = Node(
        package="grasp_preshaping",
        executable="grasp_proximity_controller_node.py",
        name="proximity_controller",
        parameters=[config],
        output="screen",
    )

    # Force Controller
    force_controller = Node(
        package="force_controller",
        executable="force_controller_node",
        name="force_controller",
        parameters=[config],
        output="screen",
    )

    # RViz
    rviz_config = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "rviz", "prosthesis.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Static TF: camera_color_optical_frame → world (identity)
    # Required by segmentation bridge to transform clicks from the twist
    # propagation frame to the cloud frame.  In the real system this comes
    # from the camera extrinsics calibration.
    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_optical_tf",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "camera_color_optical_frame",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            config_arg,
            rviz_arg,
            tsdf_arg,
            camera_tf,
            mock_cloud,
            mock_odom,
            mock_emg,
            gtsam_tracker,
            keyframe_buffer,
            cross_camera_features,
            tsdf_fusion,
            segmentation_bridge,
            twist_propagation,
            preshaping_service,
            proximity_controller,
            force_controller,
            rviz,
        ]
    )
