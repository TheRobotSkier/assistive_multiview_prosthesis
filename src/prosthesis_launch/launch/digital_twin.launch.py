"""Digital Twin launch — full pipeline with cameras, segmentation, hand model, and ChArUco tracking.

Launches all nodes needed for a complete digital twin test:

 1. Jetson RealSense camera topics, optional local RealSense launch, or mock cloud publisher
 2. Static TF: marker_map -> world identity anchor
 3. Pointcloud merger       — TF-transforms both clouds into world frame, fuses to /fused_pointcloud
 4. Pointcloud relay         — /fused_pointcloud -> /segmentation/input_cloud
 5. Segmentation ROS bridge  — HTTP inference client
 6. Click relay              — forwards RViz clicks to segmentation seeds
 7. OpenVINS hand tracker    — derives world→wrist_link from arm camera TF
 9. Cloud snapshot node      — freezes segmented cloud for RViz
10. Twist propagation        — detects hand->object collision (active)
11. Grasp preshaping service — C++/Rust FFI bridge
12. Position grasp controller — directly executes planned closures in the twin
13. Pipeline manager         — state machine orchestrator
14. Hand pose publisher      — reads TF, publishes /hand_pose
15. Hand URDF (digital twin) — robot_state_publisher (wrist_link root, no wrist joint)
16. Optional RViz            — digital_twin.rviz config
17. Joint state publisher    — publishes default joint config (gui variant for manual control)

Usage:
  ros2 launch prosthesis_launch digital_twin.launch.py
  ros2 launch prosthesis_launch digital_twin.launch.py gui:=true
  ros2 launch prosthesis_launch digital_twin.launch.py camera:=true launch_cameras:=true
"""

import os

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


def _launch_setup(context, *args, **kwargs):
    camera_enabled = LaunchConfiguration("camera").perform(context).lower() == "true"
    launch_cameras_enabled = LaunchConfiguration("launch_cameras").perform(context).lower() == "true"
    gui_enabled = LaunchConfiguration("gui").perform(context).lower() == "true"
    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower() == "true"
    mock_emg_enabled = LaunchConfiguration("mock_emg").perform(context).lower() == "true"
    config_file = LaunchConfiguration("config_file")
    inference_url = LaunchConfiguration("inference_url")

    world_frame = "world"
    marker_map_frame = "marker_map"

    if camera_enabled:
        scene_cloud_topic = "/fused_pointcloud"
        relay_input_topic = "/fused_pointcloud"
        cam1_frame = "head_d435i_head_depth_optical_frame"
        cam1_link = "head_d435i_head_link"
        cam1_color_frame = "head_d435i_head_color_optical_frame"
        cam2_link_frame = "arm_d435i_arm_link"
        cam2_color_frame = "arm_d435i_arm_color_optical_frame"
    else:
        scene_cloud_topic = "/camera/depth/color/points"
        relay_input_topic = "/camera/depth/color/points"
        cam1_frame = "camera_depth_optical_frame"
        cam1_link = "camera_link"
        cam1_color_frame = "camera_color_optical_frame"
        cam2_link_frame = "camera_link"
        cam2_color_frame = "camera_color_optical_frame"

    nodes = []

    # ── 1. Cloud source ────────────────────────────────────────────────────
    if camera_enabled and launch_cameras_enabled:
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
    elif not camera_enabled:
        nodes.append(
            Node(
                package="pipeline_manager",
                executable="mock_cloud_publisher",
                name="mock_cloud_publisher",
                output="screen",
            )
        )

    # ── 2. Static TF: marker_map -> world identity anchor ─────────────────
    # OpenVINS publishes camera poses in marker_map. The rest of the grasping
    # stack uses world, so keep world as an identity child of marker_map.
    if camera_enabled:
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="marker_map_to_world_tf",
                arguments=[
                    "0.0", "0.0", "0.0",
                    "0.0", "0.0", "0.0", "1.0",
                    marker_map_frame,
                    world_frame,
                ],
                output="screen",
            )
        )

    # Static identity world→wrist_link keeps the hand render and /hand_pose
    # alive before camera tracking is available. When ChArUco tracking is
    # active, cam2_hand_tracker publishes dynamic TFs with newer timestamps.
    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_wrist_fallback_tf",
            arguments=[
                "0.0", "0.0", "0.0",
                "0.0", "0.0", "0.0", "1.0",
                world_frame,
                "wrist_link",
            ],
            output="screen",
        )
    )

    # ── 3. Pointcloud merger — TF-transforms both clouds into world ───────
    if camera_enabled:
        nodes.append(
            Node(
                package="camera",
                executable="pointcloud_merger_node",
                name="pointcloud_merger",
                parameters=[{
                    "cam1_topic": "/head/d435i_head/depth/color/points",
                    "cam2_topic": "/arm/d435i_arm/depth/color/points",
                    "output_topic": "/fused_pointcloud",
                    "target_frame": world_frame,
                }],
                output="screen",
            )
        )

    # ── 4. Pointcloud relay (fused -> segmentation input) ─────────────────
    nodes.append(
        Node(
            package="camera",
            executable="pointcloud_relay_node",
            name="pointcloud_relay",
            parameters=[{
                "input_topic": relay_input_topic,
                "output_topic": "/segmentation/input_cloud",
            }],
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

    # ── 7. OpenVINS hand tracker — world→wrist_link from arm camera TF ────
    if camera_enabled:
        nodes.append(
            Node(
                package="camera",
                executable="openvins_hand_tracker_node",
                name="openvins_hand_tracker",
                parameters=[{
                    "world_frame": world_frame,
                    "camera_frame": cam2_link_frame,
                    "wrist_frame": "wrist_link",
                    "wrist_cam_tx": -0.04,
                    "wrist_cam_ty": -0.01,
                    "wrist_cam_tz": 0.20,
                    "wrist_cam_roll": 1.57,
                    "wrist_cam_pitch": 0.0,
                    "wrist_cam_yaw": 1.57,
                    "publish_rate": 15.0,
                }],
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
    nodes.append(
        Node(
            package="twist_propagation",
            executable="twist_propagation_node",
            name="twist_propagation",
            parameters=[
                {
                    "active": True,
                    "input_cloud_topic": scene_cloud_topic,
                }
            ],
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

    # ── 12. Position Grasp Controller ─────────────────────────────────────
    # The digital twin has no force feedback, so execute the planner's target
    # closures directly on the same command topics that the Mia hand uses.
    nodes.append(
        Node(
            package="pipeline_manager",
            executable="digital_twin_position_grasp_controller",
            name="digital_twin_position_grasp_controller",
            parameters=[{
                "closure_scale": 1.0,
                "min_closure_amount": 0.1,
                "open_on_idle": True,
            }],
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

    # ── 13b. Mock EMG publisher (triggers pipeline without real EMG band) ─
    if mock_emg_enabled:
        nodes.append(
            Node(
                package="pipeline_manager",
                executable="mock_emg_publisher",
                name="mock_emg_publisher",
                parameters=[{
                    "grasp_gesture": 1,     # POWER
                    "release_gesture": 3,   # OPEN
                    "grasp_duration": 8.0,
                    "release_duration": 3.0,
                }],
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
            parameters=[{
                "robot_description": robot_description,
                "publish_frequency": 10.0,
            }],
            output="screen",
        )
    )

    # ── 16. RViz with digital_twin.rviz (delayed for camera init) ─────────
    if rviz_enabled:
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
    # Command-driven by the same /.../commands topics used for hardware.
    nodes.append(
        Node(
            package="pipeline_manager",
            executable="digital_twin_joint_state_publisher",
            name="digital_twin_joint_state_publisher",
            output="screen",
        )
    )

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
        DeclareLaunchArgument(
            "camera",
            default_value="true",
            description="Use real RealSense camera topics from the Jetson (default: true for digital twin)",
        ),
        DeclareLaunchArgument(
            "launch_cameras",
            default_value="false",
            description="Launch local RealSense drivers on the host instead of only subscribing to Jetson topics",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="false",
            description="Also launch joint_state_publisher_gui for manual hand control",
        ),
        DeclareLaunchArgument(
            "rviz",
            default_value="true",
            description="Launch RViz from this launch file. Compose full-stack runs set this false and start host RViz separately.",
        ),
        DeclareLaunchArgument(
            "mock_emg",
            default_value="false",
            description="Launch mock EMG publisher to trigger grasp pipeline without real EMG band",
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
