"""Full pipeline launch - all nodes with hardware.

Launches the complete prosthesis pipeline:
   1. Mia Hand driver (serial)
   2. Command bridge (forwards ros2_control topics to driver services)
   3. Wrist Dynamixel driver
   4. EMG bridge (MindRove)
   5. Pointcloud fusion (TF-transforms + merges Jetson camera clouds)
   6. Pointcloud relay (fused -> segmentation input)
   7. Odom-to-pose relay (OpenVINS odom -> /hand_pose)
   8. Segmentation ROS bridge
   9. Twist propagation target selector
  10. Grasp preshaping service
  11. Grasp proximity controller
  12. Force controller
  13. Pipeline manager (state machine)
  14. RViz

Usage:
  ros2 launch pipeline.launch.py
  ros2 launch pipeline.launch.py rviz:=false
  ros2 launch pipeline.launch.py config_file:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# Default config path: workspace-root config/prosthesis_config.yaml
_WORKSPACE_ROOT = os.path.join(
    os.path.dirname(__file__), "..", "..", "..", ".."
)
DEFAULT_CONFIG = os.path.join(_WORKSPACE_ROOT, "config", "prosthesis_config.yaml")


def generate_launch_description():
    # Launch arguments
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz"
    )
    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=DEFAULT_CONFIG,
        description="Path to prosthesis_config.yaml",
    )
    camera_arg = DeclareLaunchArgument(
        "camera", default_value="true", description="Launch RealSense camera"
    )
    mia_hand_arg = DeclareLaunchArgument(
        "mia_hand", default_value="true", description="Launch Mia Hand driver"
    )
    mia_serial_port_arg = DeclareLaunchArgument(
        "mia_serial_port",
        default_value=os.environ.get("MIA_SERIAL_PORT", "/dev/ttyUSB0"),
        description="Mia Hand serial port device",
    )
    wrist_serial_port_arg = DeclareLaunchArgument(
        "wrist_serial_port",
        default_value=os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyUSB0"),
        description="Wrist Dynamixel serial port device",
    )
    target_frame_arg = DeclareLaunchArgument(
        "target_frame",
        default_value="marker_map",
        description="Target frame for fused pointcloud (OpenVINS map frame).",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/ov_msckf_arm/odomimu",
        description="OpenVINS odometry topic for hand pose estimation.",
    )
    cam1_topic_arg = DeclareLaunchArgument(
        "cam1_topic",
        default_value="/head/d435i/head/depth/color/points",
        description="Pointcloud topic from head RealSense D435i.",
    )
    cam2_topic_arg = DeclareLaunchArgument(
        "cam2_topic",
        default_value="/arm/d435i/arm/depth/color/points",
        description="Pointcloud topic from arm RealSense D435i.",
    )
    arm_frame_arg = DeclareLaunchArgument(
        "arm_frame",
        default_value="arm_d435i_arm_depth_frame",
        description="Arm camera depth frame for hand/arm bbox removal.",
    )

    # Perception bridge parameters — wired from launch args
    cam1_topic_val = LaunchConfiguration("cam1_topic")
    cam2_topic_val = LaunchConfiguration("cam2_topic")
    arm_frame_val = LaunchConfiguration("arm_frame")

    # Pipeline Manager - state machine orchestrator
    pipeline_manager = Node(
        package="pipeline_manager",
        executable="pipeline_manager_node",
        name="pipeline_manager",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # Mia Hand Driver
    mia_hand_driver = Node(
        package="mia_hand_driver",
        executable="mia_hand_driver_node",
        name="mia_hand_driver",
        parameters=[{"serial_port": LaunchConfiguration("mia_serial_port")}],
        output="screen",
    )

    # Wrist Dynamixel Driver
    wrist_driver = Node(
        package="wrist_driver",
        executable="wrist_driver_node",
        name="wrist_driver",
        parameters=[
            LaunchConfiguration("config_file"),
            {"port": LaunchConfiguration("wrist_serial_port")},
        ],
        output="screen",
    )

    # Command Bridge — forwards *_pos_ff_controller/commands to driver services
    # and republishes joint positions as /joint_states
    command_bridge = Node(
        package="command_bridge",
        executable="command_bridge_node",
        name="command_bridge",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # EMG Bridge - MindRove gesture classifier
    emg_bridge = Node(
        package="emg_bridge",
        executable="run_classifier",
        name="emg_bridge",
        output="screen",
    )

    # Pointcloud Fusion — TF-transforms both Jetson clouds, merges, filters
    pointcloud_fusion = Node(
        package="pointcloud_fusion",
        executable="pointcloud_fusion_node",
        name="pointcloud_fusion",
        parameters=[{
            "target_frame": LaunchConfiguration("target_frame"),
            "cam1_topic": cam1_topic_val,
            "cam2_topic": cam2_topic_val,
            "arm_frame": arm_frame_val,
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

    # Pointcloud Relay — /fused_pointcloud -> /segmentation/input_cloud
    pointcloud_relay = Node(
        package="camera",
        executable="pointcloud_relay_node",
        name="pointcloud_relay",
        output="screen",
    )

    # Odom-to-Pose Relay — OpenVINS odom -> /hand_pose, /hand_twist, /hand_odom
    odom_to_pose_relay = Node(
        package="camera",
        executable="odom_to_pose_relay",
        name="odom_to_pose_relay",
        parameters=[{
            "odom_topic": LaunchConfiguration("odom_topic"),
            "pose_topic": "/hand_pose",
            "twist_topic": "/hand_twist",
            "odom_out": "/hand_odom",
        }],
        output="screen",
        arguments=["--ros-args", "--log-level", "warn"],
    )

    # Segmentation ROS bridge (talks to inference server over HTTP)
    segmentation_bridge = Node(
        package="segmentation_bridge",
        executable="segmentation_ros2_node",
        name="segmentation_bridge",
        parameters=[{
            "inference_url": "http://127.0.0.1:5678",
        }],
        output="screen",
    )

    # Grasp Preshaping Service (C++ bridge to Rust .so)
    preshaping_service = Node(
        package="grasp_preshaping",
        executable="preshaping_service_bridge_node",
        name="preshaping_service",
        output="screen",
    )

    # Grasp Proximity Controller
    proximity_controller = Node(
        package="grasp_preshaping",
        executable="grasp_proximity_controller_node.py",
        name="proximity_controller",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # Twist Propagation Target Selector
    twist_propagation = Node(
        package="twist_propagation",
        executable="twist_propagation_node",
        name="twist_propagation",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # Force Controller
    force_controller = Node(
        package="force_controller",
        executable="force_controller_node",
        name="force_controller",
        parameters=[LaunchConfiguration("config_file")],
        output="screen",
    )

    # RViz config - look in the rviz/ directory at workspace root
    rviz_config = os.path.join(
        os.path.dirname(__file__), "..", "..", "..", "..", "rviz", "prosthesis.rviz"
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # Assemble launch
    nodes = [
        pipeline_manager,
        mia_hand_driver,
        command_bridge,
        wrist_driver,
        emg_bridge,
        pointcloud_fusion,
        pointcloud_relay,
        odom_to_pose_relay,
        segmentation_bridge,
        twist_propagation,
        preshaping_service,
        proximity_controller,
        force_controller,
    ]

    # Conditional nodes - always included, can be toggled
    # (Launch system doesn't support true conditionals easily,
    #  so we include them and let the nodes handle missing hardware)

    # RViz - included by default
    nodes.append(rviz)

    return LaunchDescription(
        [
            rviz_arg,
            config_arg,
            camera_arg,
            mia_hand_arg,
            mia_serial_port_arg,
            wrist_serial_port_arg,
            target_frame_arg,
            odom_topic_arg,
            cam1_topic_arg,
            cam2_topic_arg,
            arm_frame_arg,
        ]
        + nodes
    )
