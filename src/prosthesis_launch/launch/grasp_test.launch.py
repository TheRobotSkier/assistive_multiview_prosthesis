"""Grasp test launch — end-to-end grasp pipeline for development & testing.

Launches all nodes needed for a complete grasp test:

  1. (Optional) Dual RealSense D435i cameras with IMU OR mock cloud publisher
  2. Cloud snapshot node          — freezes segmented cloud for RViz
  3. Twist propagation            — detects hand→object collision (active)
  4. Grasp preshaping service     — C++/Rust FFI bridge
  5. Grasp proximity controller   — approach & contact phase
  6. Pipeline manager             — state machine orchestrator
  7. Segmentation ROS bridge      — HTTP inference client
  8. Hand pose publisher          — reads TF, publishes /hand_pose
   9. Static TF                    — wrist_link → d435i_arm_depth_optical_frame
 10. Hand URDF                    — robot_state_publisher (xacro)
 11. Wrist Dynamixel driver
 12. RViz                         — grasp_test.rviz config
 13. (Optional) joint_state_publisher_gui for manual hand control

Usage:
  ros2 launch grasp_test.launch.py                        # mock cloud
  ros2 launch grasp_test.launch.py camera:=true           # real D435 cameras
  ros2 launch grasp_test.launch.py gui:=true              # + joint_state_publisher_gui
  ros2 launch grasp_test.launch.py camera:=true gui:=true # both
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
    """Build node list based on camera/gui launch arguments."""

    camera_enabled = LaunchConfiguration("camera").perform(context).lower() == "true"
    gui_enabled = LaunchConfiguration("gui").perform(context).lower() == "true"
    config_file = LaunchConfiguration("config_file")

    # Choose the correct cloud topic based on camera vs mock
    if camera_enabled:
        cloud_topic = "/head/d435i_head/depth/color/points"
    else:
        cloud_topic = "/camera/depth/color/points"

    nodes = []

    # ── Cloud source ────────────────────────────────────────────────────
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

    # ── Segmentation bridge (input remapped to actual cloud topic) ──────
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="segmentation_ros2_node",
            name="segmentation_bridge",
            remappings={("/segmentation/input_cloud", cloud_topic)},
            parameters=[{"inference_url": "http://127.0.0.1:5678"}],
            output="screen",
        )
    )

    # ── Cloud snapshot node ─────────────────────────────────────────────
    # Freezes the latest segmented object cloud for RViz visualization
    # (topic: /segmentation/object_cloud_snapshot).
    nodes.append(
        Node(
            package="camera",
            executable="cloud_snapshot_node",
            name="cloud_snapshot_node",
            output="screen",
        )
    )

    # ── Twist propagation (active, targeting the scene cloud) ──────────
    nodes.append(
        Node(
            package="twist_propagation",
            executable="twist_propagation_node",
            name="twist_propagation",
            parameters=[
                {
                    "active": True,
                    "input_cloud_topic": cloud_topic,
                }
            ],
            output="screen",
        )
    )

    # ── Grasp Preshaping Service ───────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="preshaping_service_bridge_node",
            name="preshaping_service",
            output="screen",
        )
    )

    # ── Grasp Proximity Controller ─────────────────────────────────────
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="grasp_proximity_controller_node.py",
            name="proximity_controller",
            parameters=[{"config_file": config_file}],
            output="screen",
        )
    )

    # ── Pipeline Manager (state machine) ────────────────────────────────
    nodes.append(
        Node(
            package="pipeline_manager",
            executable="pipeline_manager_node",
            name="pipeline_manager",
            parameters=[{"config_file": config_file}],
            output="screen",
        )
    )

    # ── Hand Pose Publisher ────────────────────────────────────────────
    # Reads the wrist_link TF transform and publishes /hand_pose
    # (geometry_msgs/PoseStamped) consumed by twist_propagation.
    nodes.append(
        Node(
            package="camera",
            executable="hand_pose_publisher",
            name="hand_pose_publisher",
            output="screen",
        )
    )

    # ── Static TF: wrist_link → d435i_arm_depth_optical_frame ────────────
    # The hand-mounted camera (D435 #2) is rigidly attached to the wrist.
    # The exact translation/rotation depends on the physical mount;
    # update these values to match the actual hardware setup.
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="wrist_to_camera2_tf",
            arguments=[
                "0.0", "0.0", "0.0",  # translation (XYZ metres)
                "0.0", "0.0", "0.0", "1.0",  # rotation (identity quaternion)
                "wrist_link",
                "d435i_arm_depth_optical_frame",
            ],
            output="screen",
        )
    )

    # ── Hand URDF via robot_state_publisher ────────────────────────────
    # Generates robot_description from mia_hand_description xacro.
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

    # ── Wrist Driver ───────────────────────────────────────────────────
    nodes.append(
        Node(
            package="wrist_driver",
            executable="wrist_driver_node",
            name="wrist_driver",
            output="screen",
        )
    )

    # ── RViz with grasp_test.rviz ──────────────────────────────────────
    # Path from installed launch dir (share/.../launch/) to workspace root:
    #   ../../../.. (5x) = /prosthesis_ws
    rviz_config = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "..", "rviz", "grasp_test.rviz"
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

    # ── (Optional) joint_state_publisher_gui ───────────────────────────
    if gui_enabled:
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        # ── Launch arguments ───────────────────────────────────────────
        DeclareLaunchArgument(
            "camera",
            default_value="false",
            description="Use real RealSense D435 cameras (default: mock cloud)",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Launch joint_state_publisher_gui for manual hand control",
        ),
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to prosthesis_config.yaml (empty = package default)",
        ),
        # ── Build node list via OpaqueFunction ─────────────────────────
        OpaqueFunction(function=_launch_setup),
    ])
