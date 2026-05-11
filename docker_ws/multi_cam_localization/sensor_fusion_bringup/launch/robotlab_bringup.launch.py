"""Unified bringup for Robotlab Jetson Orin Nano: both D435 cameras + both ICM-20948 IMUs + EKF odometry.

Launches in a single file:
  - head/d435_head  (serial 827112072033) via ExecuteProcess
  - arm/d435_arm    (serial 829212072207) via ExecuteProcess
  - Two ICM-20948 IMUs + robot_localization EKF via IncludeLaunchDescription
  - Jetson NEON pointcloud fix (delayed param set for both cameras)

ExecuteProcess is used for realsense2_camera_node (not Node) to avoid YAML
integer coercion of serial numbers -- serial_no is passed via --ros-args -p with
YAML single quotes to force string type.

Usage:
  ros2 launch sensor_fusion_bringup robotlab_bringup.launch.py
"""

from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
_REALSENSE_NODE = "/opt/ros/jazzy/lib/realsense2_camera/realsense2_camera_node"
_DEPTH_PROFILE = "640x480x15"
_COLOR_PROFILE = "640x480x15"

# IMU config paths (absolute -- Docker mounts source at /miahand_ws/src)
_IMU_CONFIG_CAM0 = "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam0.yaml"
_IMU_CONFIG_CAM1 = "/miahand_ws/src/multi_cam_localization/imu_driver/config/imu_cam1.yaml"


def _bool_str(value) -> str:
    return "true" if bool(value) else "false"


# =========================================================================
# Camera helpers (from dual_d435_native.launch.py)
# =========================================================================

def _realsense_cmd(serial: str, namespace: str, node_name: str, tf_prefix: str) -> list:
    return [
        _REALSENSE_NODE,
        "--ros-args",
        "--log-level", "info",
        "-r", f"__node:={node_name}",
        "-r", f"__ns:=/{namespace}",
        "-p", f"serial_no:={serial}",
        "-p", f"tf_prefix:={tf_prefix}",
        "-p", "camera_name:=camera",
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
    return [
        "ros2", "param", "set",
        f"/{namespace}/{node_name}",
        "pointcloud__neon_.enable",
        "true",
    ]


# =========================================================================
# Configuration loading
# =========================================================================

def _load_config(context):
    package_dir = Path(FindPackageShare("sensor_fusion_bringup").perform(context))
    config_path = package_dir / "config" / "d435_cameras.yaml"
    with open(config_path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


# =========================================================================
# Main launch setup
# =========================================================================

def _setup_launch(context, *args, **kwargs):
    data = _load_config(context)
    common = data["common"]
    head_cam = data["cameras"]["head"]
    arm_cam = data["cameras"]["arm"]

    neon_delay = float(common.get("pointcloud_neon_delay_sec", 6.0))
    apply_neon_fix = IfCondition(LaunchConfiguration("enable_pointcloud_neon_fix"))

    actions = []

    # ---- Cameras (ExecuteProcess for serial safety) ----

    # Head camera
    actions.append(ExecuteProcess(
        cmd=_realsense_cmd(
            head_cam["serial_no"],
            head_cam["namespace"],
            head_cam["name"],
            head_cam["namespace"],
        ),
        output="screen",
        emulate_tty=True,
    ))
    actions.append(TimerAction(
        period=neon_delay,
        actions=[ExecuteProcess(
            cmd=_neon_fix_cmd(head_cam["namespace"], head_cam["name"]),
            output="screen",
            condition=apply_neon_fix,
        )],
    ))

    # Arm camera
    actions.append(ExecuteProcess(
        cmd=_realsense_cmd(
            arm_cam["serial_no"],
            arm_cam["namespace"],
            arm_cam["name"],
            arm_cam["namespace"],
        ),
        output="screen",
        emulate_tty=True,
    ))
    actions.append(TimerAction(
        period=neon_delay,
        actions=[ExecuteProcess(
            cmd=_neon_fix_cmd(arm_cam["namespace"], arm_cam["name"]),
            output="screen",
            condition=apply_neon_fix,
        )],
    ))

    # ---- IMUs + EKF (via IncludeLaunchDescription) ----
    ekf_launch_path = str(
        Path(FindPackageShare("sensor_fusion_bringup").perform(context))
        / "launch"
        / "two_imus_ekf.launch.py"
    )
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch_path),
    ))

    return actions


# =========================================================================
# generate_launch_description
# =========================================================================

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg="=== Robotlab bringup: cameras + IMUs + EKF ==="),
        DeclareLaunchArgument(
            "enable_pointcloud_neon_fix",
            default_value="true",
            description="Apply Jetson pointcloud__neon_.enable fix after startup.",
        ),
        OpaqueFunction(function=_setup_launch),
    ])
