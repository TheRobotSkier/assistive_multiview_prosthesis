"""Launch two RealSense D435 cameras using native ExecuteProcess for serial safety.

Uses the same pattern as full_test_implementation: ExecuteProcess (not Node)
for realsense2_camera_node to avoid YAML integer coercion of serial numbers.
Passes serial_no via --ros-args -p with YAML single quotes to force string type.

Jetson-specific additions:
  - NEON pointcloud fix (pointcloud__neon_.enable after startup delay)
  - IMU disabled (enable_gyro:=false, enable_accel:=false)
  - 15fps depth/color profiles tuned for Jetson Orin Nano

Cameras:
  - head/d435_head  serial=823313022234
  - arm/d435_arm    serial=830213023028
"""

from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_REALSENSE_NODE = "/opt/ros/jazzy/lib/realsense2_camera/realsense2_camera_node"
_DEPTH_PROFILE = "640x480x15"
_COLOR_PROFILE = "640x480x15"


def _bool_str(value) -> str:
    return "true" if bool(value) else "false"


def _realsense_cmd(serial: str, namespace: str, node_name: str, tf_prefix: str) -> list:
    """Build ExecuteProcess cmd for one realsense2_camera_node.

    YAML single quotes around serial_no force string interpretation, working
    around the launch framework's tendency to coerce all-numeric serials to int.
    """
    return [
        _REALSENSE_NODE,
        "--ros-args",
        "--log-level", "info",
        "-r", f"__node:={node_name}",
        "-r", f"__ns:=/{namespace}",
        # YAML single quotes force string type (see module docstring)
        "-p", f"serial_no:='{serial}'",
        "-p", f"tf_prefix:='{tf_prefix}'",
        "-p", "camera_name:='camera'",
        "-p", "enable_color:=true",
        "-p", f"depth_module.depth_profile:={_DEPTH_PROFILE}",
        "-p", f"rgb_camera.color_profile:={_COLOR_PROFILE}",
        "-p", "pointcloud.enable:=true",
        "-p", "pointcloud.stream_filter:=2",
        "-p", "align_depth.enable:=true",
        "-p", "enable_infra1:=false",
        "-p", "enable_infra2:=false",
        "-p", "enable_gyro:=false",
        "-p", "enable_accel:=false",
        "-p", "unite_imu_method:=0",
        "-p", "initial_reset:=false",
    ]


def _neon_fix_cmd(namespace: str, node_name: str) -> list:
    """Build cmd for the Jetson NEON pointcloud fix (delayed param set)."""
    return [
        "ros2", "param", "set",
        f"/{namespace}/{node_name}",
        "pointcloud__neon_.enable",
        "true",
    ]


def _load_config(context):
    """Load D435 camera configuration from YAML (used for metadata/reference)."""
    package_dir = Path(FindPackageShare("sensor_fusion_bringup").perform(context))
    config_path = package_dir / "config" / "d435_cameras.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _setup_launch(context, *args, **kwargs):
    data = _load_config(context)
    common = data["common"]
    head_cam = data["cameras"]["head"]
    arm_cam = data["cameras"]["arm"]

    neon_delay = float(common.get("pointcloud_neon_delay_sec", 6.0))
    apply_neon_fix = IfCondition(LaunchConfiguration("enable_pointcloud_neon_fix"))

    actions = []

    # --- Head camera ---
    actions.append(
        ExecuteProcess(
            cmd=_realsense_cmd(
                head_cam["serial_no"],
                head_cam["namespace"],
                head_cam["name"],
                head_cam["namespace"],  # tf_prefix matches namespace
            ),
            output="screen",
            emulate_tty=True,
        )
    )
    # NEON fix for head
    actions.append(
        TimerAction(
            period=neon_delay,
            actions=[
                ExecuteProcess(
                    cmd=_neon_fix_cmd(head_cam["namespace"], head_cam["name"]),
                    output="screen",
                    condition=apply_neon_fix,
                )
            ],
        )
    )

    # --- Arm camera ---
    actions.append(
        ExecuteProcess(
            cmd=_realsense_cmd(
                arm_cam["serial_no"],
                arm_cam["namespace"],
                arm_cam["name"],
                arm_cam["namespace"],  # tf_prefix matches namespace
            ),
            output="screen",
            emulate_tty=True,
        )
    )
    # NEON fix for arm
    actions.append(
        TimerAction(
            period=neon_delay,
            actions=[
                ExecuteProcess(
                    cmd=_neon_fix_cmd(arm_cam["namespace"], arm_cam["name"]),
                    output="screen",
                    condition=apply_neon_fix,
                )
            ],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="Starting dual D435 cameras (native ExecuteProcess, serial-safe)"),
        DeclareLaunchArgument(
            "enable_pointcloud_neon_fix",
            default_value="true",
            description="Apply Jetson pointcloud__neon_.enable fix after startup.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
