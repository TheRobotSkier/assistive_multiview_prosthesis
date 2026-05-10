"""Digital Twin launch — full pipeline with cameras, segmentation, and hand model.

Launches all nodes needed for a complete digital twin test:

  1. Dual RealSense D435 cameras (or mock cloud publisher)
  2. Pointcloud fuser        — subscribes to both cameras, publishes /fused_pointcloud
  3. Cloud snapshot node      — freezes segmented cloud for RViz
  4. Pointcloud relay         — /fused_pointcloud -> /segmentation/input_cloud
  5. Segmentation ROS bridge  — HTTP inference client
  6. Click relay              — forwards RViz clicks to segmentation seeds
  7. Twist propagation        — detects hand->object collision (active)
  8. Grasp preshaping service — C++/Rust FFI bridge
  9. Grasp proximity controller
  10. Pipeline manager         — state machine orchestrator
  11. Hand pose publisher      — reads TF, publishes /hand_pose
  12. Static TF                — wrist_link -> d435_2_depth_optical_frame
  13. Hand URDF                — robot_state_publisher (xacro)
  14. Wrist Dynamixel driver   — starts even without hardware (logs warnings)
  15. RViz                     — digital_twin.rviz config
  16. Joint state publisher    — publishes default joint config (gui variant optional)

Usage:
  ros2 launch prosthesis_launch digital_twin.launch.py
  ros2 launch prosthesis_launch digital_twin.launch.py camera:=true
  ros2 launch prosthesis_launch digital_twin.launch.py gui:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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


def _launch_setup(context, *args, **kwargs):
    camera_enabled = LaunchConfiguration("camera").perform(context).lower() == "true"
    gui_enabled = LaunchConfiguration("gui").perform(context).lower() == "true"
    config_file = LaunchConfiguration("config_file")
    inference_url = LaunchConfiguration("inference_url")

    if camera_enabled:
        cloud_topic = "/cam1/d435_1/depth/color/points"
    else:
        cloud_topic = "/camera/depth/color/points"

    nodes = []

    # ── 1. Cloud source ────────────────────────────────────────────────────
    if camera_enabled:
        camera_launch_path = os.path.join(
            get_package_share_directory("camera"),
            "launch",
            "two_d435.launch.py",
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

    # ── 2. Pointcloud fuser (combines both camera streams) ────────────────
    if camera_enabled:
        nodes.append(
            Node(
                package="camera",
                executable="pointcloud_fuser_node",
                name="pointcloud_fuser",
                parameters=[{
                    "cam1_topic": "/cam1/d435_1/depth/color/points",
                    "cam2_topic": "/cam2/d435_2/depth/color/points",
                    "output_topic": "/fused_pointcloud",
                }],
                output="screen",
            )
        )

    # ── 3. Pointcloud relay (fused -> segmentation input) ─────────────────
    nodes.append(
        Node(
            package="camera",
            executable="pointcloud_relay_node",
            name="pointcloud_relay",
            output="screen",
        )
    )

    # ── 4. Segmentation bridge ────────────────────────────────────────────
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="segmentation_ros2_node",
            name="segmentation_bridge",
            parameters=[{"inference_url": inference_url}],
            output="screen",
        )
    )

    # ── 5. Click relay ────────────────────────────────────────────────────
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="demo_click_relay_node",
            name="click_relay",
            output="screen",
        )
    )

    # ── 6. Cloud snapshot node ────────────────────────────────────────────
    nodes.append(
        Node(
            package="camera",
            executable="cloud_snapshot_node",
            name="cloud_snapshot_node",
            output="screen",
        )
    )

    # ── 7. Twist propagation ──────────────────────────────────────────────
    nodes.append(
        Node(
            package="twist_propagation",
            executable="twist_propagation_node",
            name="twist_propagation",
            parameters=[
                {
                    "active": False,
                    "input_cloud_topic": cloud_topic,
                }
            ],
            output="screen",
        )
    )

    # ── 8. Grasp Preshaping Service ──────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="preshaping_service_bridge_node",
            name="preshaping_service",
            parameters=[{
                "camera_frames": [
                    "cam1_d435_1_color_optical_frame",
                    "cam2_d435_2_color_optical_frame",
                ],
                "preshaping_closure_fraction": 0.3,
                "min_closure_amount": 0.1,
                "publish_initial_commands": False,
            }],
            output="screen",
        )
    )

    # ── 9. Grasp Proximity Controller ────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="grasp_proximity_controller_node.py",
            name="proximity_controller",
            parameters=[{"config_file": config_file}],
            output="screen",
        )
    )

    # ── 10. Pipeline Manager ──────────────────────────────────────────────
    nodes.append(
        Node(
            package="pipeline_manager",
            executable="pipeline_manager_node",
            name="pipeline_manager",
            parameters=[{"config_file": config_file}],
            output="screen",
        )
    )

    # ── 11. Hand Pose Publisher ──────────────────────────────────────────
    nodes.append(
        Node(
            package="camera",
            executable="hand_pose_publisher",
            name="hand_pose_publisher",
            output="screen",
        )
    )

    # ── 12. Static TF: wrist_link -> d435_2_depth_optical_frame ─────────
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="wrist_to_camera2_tf",
            arguments=[
                "0.0", "0.0", "0.0",
                "0.0", "0.0", "0.0", "1.0",
                "wrist_link",
                "d435_2_depth_optical_frame",
            ],
            output="screen",
        )
    )

    # ── 13. Hand URDF via robot_state_publisher ──────────────────────────
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([
                FindPackageShare("mia_hand_description"),
                "urdf",
                "mia_hand_description.urdf.xacro",
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

    # ── 14. Wrist Driver ─────────────────────────────────────────────────
    nodes.append(
        Node(
            package="wrist_driver",
            executable="wrist_driver_node",
            name="wrist_driver",
            output="screen",
        )
    )

    # ── 15. RViz with digital_twin.rviz ──────────────────────────────────
    rviz_config = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "rviz", "digital_twin.rviz"
    )
    nodes.append(
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            output="screen",
        )
    )

    # ── 16. Joint state publisher ────────────────────────────────────────
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
