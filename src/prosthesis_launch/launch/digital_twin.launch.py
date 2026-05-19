"""Digital Twin launch — full pipeline with cameras, segmentation, hand model, and ChArUco tracking.

Launches all nodes needed for a complete digital twin test:

 1. Dual RealSense D435i cameras with IMU (or mock cloud publisher)
 2. Static TF: d435i_head_depth_optical_frame -> world (root)
 3. Pointcloud fusion        — TF-transforms both clouds to world frame, merges, filters, publishes /fused_pointcloud
 4. Pointcloud relay         — /fused_pointcloud -> /segmentation/input_cloud
 5. Segmentation ROS bridge  — HTTP inference client
 6. Click relay              — forwards RViz clicks to segmentation seeds
 7. ChArUco TF node          — detects board, publishes charuco_board→cam*_link transforms
 8. Cam2 hand tracker        — composes ChArUco TFs → publishes world→wrist_link dynamically
 9. Cloud snapshot node      — freezes segmented cloud for RViz
10. Twist propagation        — detects hand->object collision (active)
11. Grasp preshaping service — C++/Rust FFI bridge
12. Grasp proximity controller
13. Pipeline manager         — state machine orchestrator
14. Hand pose publisher      — reads TF, publishes /hand_pose
15. Hand URDF (digital twin) — robot_state_publisher (wrist_link root, no wrist joint)
16. RViz                     — digital_twin.rviz config
17. Joint state publisher    — publishes default joint config (gui variant for manual control)

Usage:
  ros2 launch prosthesis_launch digital_twin.launch.py
  ros2 launch prosthesis_launch digital_twin.launch.py gui:=true
"""

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _load_twist_params_from_config(config_file_path):
    """Load twist_propagation parameters from prosthesis_config.yaml."""
    if config_file_path and os.path.exists(config_file_path):
        with open(config_file_path, "r", encoding="utf-8") as f:
            cfg = yaml.safe_load(f)
        return dict(cfg.get("twist_propagation", {}).get("ros__parameters", {}))
    return {}


# Default config path: workspace-root config/prosthesis_config.yaml
_WORKSPACE_ROOT = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."
)
DEFAULT_CONFIG = os.path.join(_WORKSPACE_ROOT, "config", "prosthesis_config.yaml")


def _launch_setup(context, *args, **kwargs):
    camera_enabled = LaunchConfiguration("camera").perform(context).lower() == "true"
    gui_enabled = LaunchConfiguration("gui").perform(context).lower() == "true"
    config_file_raw = LaunchConfiguration("config_file").perform(context)
    config_file = config_file_raw if config_file_raw else DEFAULT_CONFIG
    inference_url = LaunchConfiguration("inference_url").perform(context)

    if camera_enabled:
        cloud_topic = "/head/d435i_head/depth/color/points"
        cam1_frame = "d435i_head_depth_optical_frame"
        cam1_link = "d435i_head_link"
        cam1_color_frame = "d435i_head_color_optical_frame"
        cam2_link_frame = "d435i_arm_link"
        cam2_color_frame = "d435i_arm_color_optical_frame"
    else:
        cloud_topic = "/camera/depth/color/points"
        cam1_frame = "camera_depth_optical_frame"
        cam1_link = "camera_link"
        cam1_color_frame = "camera_color_optical_frame"
        cam2_link_frame = "camera_link"
        cam2_color_frame = "camera_color_optical_frame"

    nodes = []

    # ── 1. Cloud source ────────────────────────────────────────────────────
    if camera_enabled:
        camera_launch_path = os.path.join(
            get_package_share_directory("sensor_fusion_bringup"),
            "launch",
            "dual_d435i.launch.py",
        )
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(camera_launch_path),
            )
        )
    else:
        nodes.append(
            Node(
                package="pipeline_manager",
                executable="mock_cloud_publisher",
                name="mock_cloud_publisher",
                output="screen",
            )
        )

    # ── 2. Static TF: cam1 depth frame -> world (roots URDF in camera 1) ───
    # Since camera 1 is the stationary reference, this connects the TF tree.
    # cam2's position comes from ChArUco tracking → cam2_hand_tracker.
    # RViz uses d435i_head_depth_optical_frame as fixed frame.
    if camera_enabled:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="cam1_to_world_tf",
                arguments=[
                    "0.0", "0.0", "0.0",
                    "0.0", "0.0", "0.0", "1.0",
                    cam1_frame,
                    "world",
                ],
                output="screen",
            )
        )
        # Static identity world→wrist_link — keeps the TF chain connected
        # at all timestamps so RViz can render cam2 pointclouds even without
        # the ChArUco board. When ChArUco tracking is active, cam2_hand_tracker
        # publishes dynamic TFs with newer timestamps that take precedence.
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_wrist_fallback_tf",
                arguments=[
                    "0.0", "0.0", "0.0",
                    "0.0", "0.0", "0.0", "1.0",
                    "world",
                    "wrist_link",
                ],
                output="screen",
            )
        )

    # ── 3. Pointcloud fusion — TF-transforms both clouds to world, merges, filters ──
    # Replaces the old pointcloud_merger_node which fused in cam1 frame.
    # This node transforms BOTH clouds to world frame, applies distance filtering,
    # hand/arm bbox removal, and voxel downsampling.
    if camera_enabled:
        nodes.append(
            Node(
                package='pointcloud_fusion',
                executable='pointcloud_fusion_node',
                name='pointcloud_fusion',
                parameters=[{
                    'target_frame': 'world',
                    'cam1_topic': '/head/d435i_head/depth/color/points',
                    'cam2_topic': '/arm/d435i_arm/depth/color/points',
                    'arm_frame': 'wrist_link',
                    'max_distance': 2.0,
                    'voxel_size': 0.005,
                    'bbox_min': [-0.30, -0.10, -0.10],
                    'bbox_max': [0.22, 0.10, 0.12],
                    'enable_downsampling': True,
                    'enable_distance_filter': True,
                    'enable_hand_removal': True,
                }],
                output='screen',
            )
        )

    # ── 4. Pointcloud relay (fused -> segmentation input) ─────────────────
    nodes.append(
        Node(
            package="camera",
            executable="pointcloud_relay_node",
            name="pointcloud_relay",
            output="screen",
        )
    )

    # ── 5. Segmentation bridge ────────────────────────────────────────────
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="segmentation_ros2_node",
            name="segmentation_bridge",
            parameters=[{"inference_url": inference_url}],
            output="screen",
        )
    )

    # ── 6. Click relay ────────────────────────────────────────────────────
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="demo_click_relay_node",
            name="click_relay",
            output="screen",
        )
    )

    # ── 7. ChArUco TF node — detects board, publishes charuco_board→cam*_link ─
    # This is the foundation for cam2 tracking: gives us the board pose
    # relative to each camera so the cam2_hand_tracker can compose world→cam2.
    if camera_enabled:
        nodes.append(
            Node(
                package="camera",
                executable="charuco_tf_node",
                name="charuco_tf_node",
                parameters=[{
                    "board_frame": "charuco_board",
                    # Override defaults to match our tf_prefix + camera_name setup
                    "cam1_frame_id": cam1_color_frame,
                    "cam1_link_frame": cam1_link,
                    "cam2_frame_id": cam2_color_frame,
                    "cam2_link_frame": cam2_link_frame,
                }],
                output="screen",
            )
        )

    # ── 8. Cam2 Hand Tracker — world→wrist_link from ChArUco tracking ─────
    # Composes: cam1→world (static) + cam1→board (ChArUco) + board→cam2 (ChArUco)
    # → world→cam2 → apply wrist offset → world→wrist_link TF.
    if camera_enabled:
        nodes.append(
            Node(
                package="camera",
                executable="cam2_hand_tracker_node",
                name="cam2_hand_tracker",
                parameters=[{
                    "board_frame": "charuco_board",
                    "cam1_link": cam1_link,
                    "cam1_depth_frame": cam1_frame,
                    "cam2_link": cam2_link_frame,
                    "world_frame": "world",
                    # wrist→cam2 mounting offset (same as static TF below)
                    "wrist_cam2_tx": -0.04,
                    "wrist_cam2_ty": -0.01,
                    "wrist_cam2_tz": 0.20,
                    "wrist_cam2_roll": 1.57,
                    "wrist_cam2_pitch": 0.0,
                    "wrist_cam2_yaw": 1.57,
                    "publish_rate": 15.0,
                }],
                output="screen",
            )
        )

    # ── 8b. Static TF: wrist_link → d435i_arm_link (hand-mounted camera) ─
    # Needed for the pointcloud merger's TF chain: cam2cloud → cam2link →
    # wrist_link → world → cam1frame.  Also used by RViz to display cam2's
    # pointcloud in the hand frame.
    #   translation: (-0.04, -0.01, 0.20) — camera behind palm
    #   rotation:    (1.57, 0.0, 1.57)    — aligns camera Z forward with wrist +X
    if camera_enabled:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="wrist_to_camera2_tf",
                arguments=[
                    "-0.04", "-0.01", "0.20",
                    "1.57", "0.0", "1.57",
                    "wrist_link",
                    cam2_link_frame,
                ],
                output="screen",
            )
        )

    # ── 9. Cloud snapshot node ────────────────────────────────────────────
    nodes.append(
        Node(
            package="camera",
            executable="cloud_snapshot_node",
            name="cloud_snapshot_node",
            output="screen",
        )
    )

    # ── 10. Twist propagation ──────────────────────────────────────────────
    # Load full parameter set from config, overlay overrides.
    twist_params = _load_twist_params_from_config(config_file)
    twist_params["active"] = False
    twist_params["input_cloud_topic"] = "/fused_pointcloud"

    nodes.append(
        Node(
            package="twist_propagation",
            executable="twist_propagation_node",
            name="twist_propagation",
            parameters=[twist_params],
            output="screen",
        )
    )

    # ── 11. Grasp Preshaping Service ──────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="preshaping_service_bridge_node",
            name="preshaping_service",
            parameters=[{
                "camera_frames": [
                    cam1_color_frame,
                    cam2_color_frame,
                ],
                "preshaping_closure_fraction": 0.3,
                "min_closure_amount": 0.1,
                "publish_initial_commands": False,
            }],
            output="screen",
        )
    )

    # ── 12. Grasp Proximity Controller ────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="grasp_proximity_controller_node.py",
            name="proximity_controller",
            parameters=[{"config_file": config_file}],
            output="screen",
        )
    )

    # ── 13. Pipeline Manager ──────────────────────────────────────────────
    nodes.append(
        Node(
            package="pipeline_manager",
            executable="pipeline_manager_node",
            name="pipeline_manager",
            parameters=[{"config_file": config_file}],
            output="screen",
        )
    )

    # ── 14. Hand Pose Publisher ──────────────────────────────────────────
    nodes.append(
        Node(
            package="camera",
            executable="hand_pose_publisher",
            name="hand_pose_publisher",
            output="screen",
        )
    )

    # ── 15. Hand URDF (digital twin) via robot_state_publisher ────────────
    # Uses mia_hand_digital_twin.urdf.xacro: wrist_link as root, NO world link
    # and NO wrist_rotation joint. world→wrist_link is provided dynamically
    # by cam2_hand_tracker_node from ChArUco board tracking.
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("mia_hand_ros2_control"),
                "description", "urdf",
                "mia_hand_digital_twin.urdf.xacro",
            ]),
            " laterality:=right",
            " prefix:=",
            TextSubstitution(text=""),
        ]),
        value_type=str,
    )

    nodes.append(
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        )
    )

    # ── 16. RViz with digital_twin.rviz (delayed for camera init) ─────────
    rviz_config = os.path.abspath(os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "..", "rviz", "digital_twin.rviz"
    ))
    # Delay RViz so camera frames exist before it opens (otherwise falls to 'map')
    rviz = TimerAction(period=8.0, actions=[
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    ])
    nodes.append(rviz)

    # ── 17. Joint state publisher ────────────────────────────────────────
    if gui_enabled:
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            )
        )
    else:
        nodes.append(
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "camera",
            default_value="true",
            description="Use real RealSense D435 cameras (default: true for digital twin)",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Launch joint_state_publisher_gui for manual hand control",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to prosthesis_config.yaml (empty = package default)",
        ),
        DeclareLaunchArgument(
            "inference_url",
            default_value="http://127.0.0.1:5678",
            description="Segmentation inference server URL",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
