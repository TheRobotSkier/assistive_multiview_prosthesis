"""Full pipeline launch - all nodes with hardware.

Launches the complete prosthesis pipeline:
   1. Mia Hand driver (serial)
   2. Command bridge (forwards ros2_control topics to driver services)
   3. Wrist Dynamixel driver
   4. EMG bridge (MindRove)
   5. Haptic bridge and controller
   6. Pointcloud fusion (TF-transforms + merges Jetson camera clouds)
   7. Odom-to-pose relay (OpenVINS odom -> /hand_pose)
   8. Segmentation ROS bridge (subscribes directly to /fused_pointcloud)
   9. Twist propagation target selector
  11. Grasp preshaping service
  12. Grasp proximity controller
  13. Force controller
  14. Pipeline manager (state machine)
  15. RViz

Usage:
  ros2 launch prosthesis_launch pipeline.launch.py
  ros2 launch prosthesis_launch pipeline.launch.py rviz:=false
  ros2 launch prosthesis_launch pipeline.launch.py mia_hand:=false wrist:=false
"""

import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Launch files run from the installed package share path, while the dev
# container bind-mounts runtime config at /prosthesis_ws/config.
DEFAULT_CONFIG = os.environ.get(
    "PROSTHESIS_CONFIG",
    "/prosthesis_ws/config/prosthesis_config.yaml",
)
DEFAULT_RVIZ_CONFIG = os.environ.get(
    "PROSTHESIS_RVIZ_CONFIG",
    "/prosthesis_ws/rviz/prosthesis.rviz",
)


def _as_bool(context, name: str) -> bool:
    value = LaunchConfiguration(name).perform(context).lower()
    return value in ("1", "true", "yes", "on")


def _load_config(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def _node_params(config: dict, node_name: str) -> dict:
    """Return only the parameters for one node from the mixed central config.

    config/prosthesis_config.yaml also contains flat reference sections, so it
    cannot be passed directly to ROS as a params file.
    """
    section = config.get(node_name, {})
    if not isinstance(section, dict):
        return {}
    ros_params = section.get("ros__parameters")
    if isinstance(ros_params, dict):
        return dict(ros_params)
    return dict(section)


def _launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration("config_file").perform(context)
    config = _load_config(config_file)

    target_frame = LaunchConfiguration("target_frame").perform(context)
    odom_topic = LaunchConfiguration("odom_topic").perform(context)
    cam1_topic = LaunchConfiguration("cam1_topic").perform(context)
    cam2_topic = LaunchConfiguration("cam2_topic").perform(context)
    arm_frame = LaunchConfiguration("arm_frame").perform(context)
    mia_serial_port = LaunchConfiguration("mia_serial_port").perform(context)
    wrist_serial_port = LaunchConfiguration("wrist_serial_port").perform(context)
    haptic_bt_addr = LaunchConfiguration("haptic_bt_addr1").perform(context)
    roi_radius = LaunchConfiguration("roi_radius").perform(context)
    inference_url = LaunchConfiguration("inference_url").perform(context)
    camera_mount = LaunchConfiguration("camera_mount").perform(context)
    mounts_config = LaunchConfiguration("mounts_config").perform(context)
    mounts_link_frame = LaunchConfiguration("mounts_link_frame").perform(context)
    tf_diagnostics = LaunchConfiguration("tf_diagnostics").perform(context)
    model_dir = LaunchConfiguration("model_dir").perform(context)

    nodes = []

    # Pipeline Manager - state machine orchestrator
    nodes.append(
        Node(
            package="pipeline_manager",
            executable="pipeline_manager_node",
            name="pipeline_manager",
            parameters=[_node_params(config, "pipeline_manager")],
            output="screen",
        )
    )

    if _as_bool(context, "mia_hand"):
        nodes.extend(
            [
                Node(
                    package="mia_hand_driver",
                    executable="mia_hand_driver_node",
                    name="mia_hand_driver",
                    parameters=[{"serial_port": mia_serial_port}],
                    output="screen",
                ),
                Node(
                    package="command_bridge",
                    executable="command_bridge_node",
                    name="command_bridge",
                    parameters=[_node_params(config, "command_bridge")],
                    output="screen",
                ),
            ]
        )

    if _as_bool(context, "wrist"):
        hardware = config.get("hardware", {}) if isinstance(config.get("hardware", {}), dict) else {}
        nodes.append(
            Node(
                package="wrist_driver",
                executable="wrist_driver_node",
                name="wrist_driver",
                parameters=[
                    {
                        "port": wrist_serial_port,
                        "baudrate": hardware.get("wrist_baudrate", 57600),
                        "motor_id": hardware.get("wrist_motor_id", 1),
                    }
                ],
                output="screen",
            )
        )

    if _as_bool(context, "emg"):
        nodes.extend(
            [
                Node(
                    package="emg_bridge",
                    executable="run_classifier",
                    name="emg_bridge",
                    arguments=["--model-dir", model_dir],
                    output="screen",
                ),
                Node(
                    package="emg_bridge",
                    executable="emg_grasp_controller",
                    name="emg_grasp_controller",
                    parameters=[_node_params(config, "emg_grasp_controller")],
                    output="screen",
                ),
            ]
        )

    if _as_bool(context, "haptic"):
        nodes.extend(
            [
                Node(
                    package="haptic_bridge",
                    executable="bridge_node",
                    name="haptic_bridge",
                    additional_env={"HAPTIC_BT_ADDR1": haptic_bt_addr},
                    output="screen",
                ),
                Node(
                    package="haptic_bridge",
                    executable="haptic_controller_node",
                    name="haptic_controller",
                    parameters=[_node_params(config, "haptic_controller")],
                    output="screen",
                ),
            ]
        )

    if _as_bool(context, "camera"):
        fusion_params = _node_params(config, "pointcloud_fusion")
        fusion_params.update(
            {
                "target_frame": target_frame,
                "cam1_topic": cam1_topic,
                "cam2_topic": cam2_topic,
                "arm_frame": arm_frame,
                "mounts_config_path": mounts_config,
                "active_mount": camera_mount,
            }
        )
        camera_nodes = []
        if _as_bool(context, "camera_tf_bridge"):
            camera_nodes.append(
                Node(
                    package="camera",
                    executable="openvins_realsense_tf_bridge_node",
                    name="openvins_realsense_tf_bridge",
                    parameters=[_node_params(config, "openvins_realsense_tf_bridge")],
                    output="screen",
                )
            )
        # Publish camera mount TFs (palm_frame, bounding boxes, grasp contact)
        # so the fusion node can look up pruning box frames.
        if mounts_config:
            mounts_script = os.path.join(
                os.path.dirname(__file__), "..", "..", "..",
                "src", "sensor_fusion_bringup", "scripts",
                "publish_camera_mounts.py")
            if not os.path.isfile(mounts_script):
                # Installed layout
                import ament_index_python
                try:
                    share = ament_index_python.get_package_share_directory(
                        "sensor_fusion_bringup")
                    mounts_script = os.path.join(
                        share, "scripts", "publish_camera_mounts.py")
                except Exception:
                    mounts_script = ""
            if mounts_script and os.path.isfile(mounts_script):
                mounts_cmd = ["python3", mounts_script,
                              "--mount", camera_mount,
                              "--config", mounts_config]
                if mounts_link_frame:
                    mounts_cmd.extend(["--link-frame", mounts_link_frame])
                camera_nodes.append(
                    ExecuteProcess(
                        cmd=mounts_cmd,
                        name="camera_mount_tf_publisher",
                        output="screen",
                    )
                )
            else:
                print(f"[pipeline] WARNING: camera mount TF publisher script not found "
                      f"(tried source-tree and installed layouts). "
                      f"mounts_config={mounts_config!r}, mounts_script={mounts_script!r}. "
                      f"Camera mount TFs (palm_frame, grasp_contact_frame, etc.) will NOT "
                      f"be published. Rebuild prosthesis_launch and sensor_fusion_bringup.")
        # TF pipeline diagnostics — logs clear one-line summaries of which TF
        # chains are healthy vs. disconnected (OpenVINS vs. camera mounts).
        if _as_bool(context, "tf_diagnostics"):
            diag_script = os.path.join(
                os.path.dirname(__file__), "..", "..", "..",
                "src", "sensor_fusion_bringup", "scripts",
                "tf_pipeline_diagnostics.py")
            if not os.path.isfile(diag_script):
                import ament_index_python
                try:
                    share = ament_index_python.get_package_share_directory(
                        "sensor_fusion_bringup")
                    diag_script = os.path.join(share, "scripts",
                                               "tf_pipeline_diagnostics.py")
                except Exception:
                    diag_script = ""
            if diag_script and os.path.isfile(diag_script):
                camera_nodes.append(
                    ExecuteProcess(
                        cmd=["python3", diag_script],
                        name="tf_pipeline_diagnostics",
                        output="screen",
                    )
                )
            else:
                print(f"[pipeline] WARNING: TF diagnostics script not found "
                      f"(tried source-tree and installed layouts). "
                      f"diag_script={diag_script!r}.")
        camera_nodes.extend(
            [
                Node(
                    package="pointcloud_fusion",
                    executable="pointcloud_fusion_node",
                    name="pointcloud_fusion",
                    parameters=[fusion_params],
                    output="screen",
                ),
                Node(
                    package="camera",
                    executable="odom_to_pose_relay",
                    name="odom_to_pose_relay",
                    parameters=[
                        {
                            "odom_topic": odom_topic,
                            "pose_topic": "/hand_pose",
                            "twist_topic": "/hand_twist",
                            "odom_out": "/hand_odom",
                        }
                    ],
                    output="screen",
                    arguments=["--ros-args", "--log-level", "warn"],
                ),
                # OpenVINS odometry-to-TF relay: publishes marker_map -> *_imu
                # from odom messages so the host doesn't depend on Jetson /tf.
                Node(
                    package="camera",
                    executable="openvins_odom_tf_relay",
                    name="openvins_odom_tf_relay",
                    parameters=[_node_params(config, "openvins_odom_tf_relay")],
                    output="screen",
                    arguments=["--ros-args", "--log-level", "warn"],
                ),
            ]
        )
        nodes.extend(camera_nodes)

    # Segmentation ROS bridge (talks to inference server over HTTP)
    # Subscribes directly to /fused_pointcloud via remapping.
    nodes.append(
        Node(
            package="segmentation_bridge",
            executable="segmentation_ros2_node",
            name="segmentation_bridge",
            remappings={("/segmentation/input_cloud", "/fused_pointcloud")},
            parameters=[{"inference_url": inference_url, "roi_radius_m": float(roi_radius)}],
            output="screen",
        )
    )

    # Twist Propagation Target Selector
    nodes.append(
        Node(
            package="twist_propagation",
            executable="twist_propagation_node",
            name="twist_propagation",
            parameters=[_node_params(config, "twist_propagation")],
            output="screen",
        )
    )

    # Grasp Preshaping Service (C++ bridge to Rust .so)
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="preshaping_service_bridge_node",
            name="preshaping_service",
            parameters=[_node_params(config, "preshaping_service")],
            output="screen",
        )
    )

    # Grasp Proximity Controller
    nodes.append(
        Node(
            package="grasp_preshaping",
            executable="grasp_proximity_controller_node.py",
            name="proximity_controller",
            parameters=[_node_params(config, "proximity_controller")],
            output="screen",
        )
    )

    if _as_bool(context, "mia_hand"):
        nodes.append(
            Node(
                package="force_controller",
                executable="force_controller_node",
                name="force_controller",
                parameters=[_node_params(config, "force_controller")],
                output="screen",
            )
        )

    if _as_bool(context, "rviz"):
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", DEFAULT_RVIZ_CONFIG],
                output="screen",
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz", default_value="true", description="Launch RViz"
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=DEFAULT_CONFIG,
                description="Path to prosthesis_config.yaml",
            ),
            DeclareLaunchArgument(
                "camera",
                default_value="true",
                description="Launch host perception bridge nodes for Jetson camera topics",
            ),
            DeclareLaunchArgument(
                "camera_tf_bridge",
                default_value="true",
                description="Bridge OpenVINS camera frames into the RealSense TF trees.",
            ),
            DeclareLaunchArgument(
                "mia_hand",
                default_value="true",
                description="Launch Mia Hand driver and force controller",
            ),
            DeclareLaunchArgument(
                "wrist", default_value="true", description="Launch wrist Dynamixel driver"
            ),
            DeclareLaunchArgument(
                "emg", default_value="true", description="Launch EMG classifier bridge"
            ),
            DeclareLaunchArgument(
                "haptic", default_value="true", description="Launch haptic bridge and controller"
            ),
            DeclareLaunchArgument(
                "haptic_bt_addr1",
                default_value=os.environ.get("HAPTIC_BT_ADDR1", "842E1409E14E"),
                description="Bluetooth address for the Vibro8 haptic band",
            ),
            DeclareLaunchArgument(
                "mia_serial_port",
                default_value=os.environ.get("MIA_SERIAL_PORT", "/dev/ttyMiaHand"),
                description="Mia Hand serial port device",
            ),
            DeclareLaunchArgument(
                "wrist_serial_port",
                default_value=os.environ.get("WRIST_SERIAL_PORT", "/dev/ttyDynamixel"),
                description="Wrist Dynamixel serial port device",
            ),
            DeclareLaunchArgument(
                "target_frame",
                default_value="marker_map",
                description="Target frame for fused pointcloud (OpenVINS map frame).",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/ov_msckf_arm/odomimu",
                description="OpenVINS odometry topic for hand pose estimation.",
            ),
            DeclareLaunchArgument(
                "cam1_topic",
                default_value="/head/d435i_head/depth/color/points",
                description="Pointcloud topic from head RealSense D435i.",
            ),
            DeclareLaunchArgument(
                "cam2_topic",
                default_value="/arm/d435i_arm/depth/color/points",
                description="Pointcloud topic from arm RealSense D435i.",
            ),
            DeclareLaunchArgument(
                "arm_frame",
                default_value="arm_d435i_arm_depth_frame",
                description="Arm camera depth frame for hand/arm bbox removal.",
            ),
            DeclareLaunchArgument(
                "camera_mount",
                default_value="8_cm_cam_mount",
                description="Camera mount name for publish_camera_mounts.py and pruning boxes.",
            ),
            DeclareLaunchArgument(
                "mounts_config",
                default_value="/prosthesis_ws/src/sensor_fusion_bringup/config/camera_mounts.yaml",
                description="Path to camera_mounts.yaml (empty = skip mount TF publisher).",
            ),
            DeclareLaunchArgument(
                "mounts_link_frame",
                default_value="arm_d435i_arm_link",
                description="TF frame to anchor the camera mounts tree under (empty = use 'world').",
            ),
            DeclareLaunchArgument(
                "inference_url",
                default_value="http://127.0.0.1:5678",
                description="Segmentation inference server URL.",
            ),
            DeclareLaunchArgument(
                "roi_radius",
                default_value="0.1",
                description="ROI crop radius (m) for pre-inference point cloud filtering. "
                            "Set <=0 to disable.",
            ),
            DeclareLaunchArgument(
                "model_dir",
                default_value="/app/models",
                description="Directory containing trained EMG classifier models.",
            ),
            DeclareLaunchArgument(
                "tf_diagnostics",
                default_value="true",
                description="Launch lightweight TF diagnostics node that logs "
                            "OpenVINS and camera-mount chain health.",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
