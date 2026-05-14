#!/usr/bin/env python3
"""IMU dead reckoning test launch file.

Auto-detects connected RealSense cameras via rs-enumerate-devices,
starts one realsense2_camera node per camera (up to 2), then starts
imu_dead_reckoning_node with a 5 s delay.

Cameras are assigned cam1/cam2 in enumeration order. D435I (name ends
in 'I') -> built-in IMU on /camN/camN/imu.  D435 -> external IMU assumed
on /camN/data_raw.
"""
from __future__ import annotations

import json
import subprocess
from pathlib import Path

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

RS_ENUM_CMD = "/opt/ros/jazzy/bin/rs-enumerate-devices"
MAX_CAMERAS = 2


def _detect_cameras() -> list[dict]:
    """Run rs-enumerate-devices -s, return list of {serial, device_name, has_builtin_imu}.

    Filters out blank lines and lines starting with whitespace (sub-info / USB paths).
    Up to MAX_CAMERAS devices are returned; extras are ignored with a warning.
    """
    try:
        result = subprocess.run(
            [RS_ENUM_CMD, "-s"],
            capture_output=True, text=True, timeout=10,
        )
        raw_lines = result.stdout.splitlines()
    except Exception as exc:  # noqa: BLE001
        print(f"[imu_test] WARNING: rs-enumerate-devices failed: {exc}")
        return []

    cameras: list[dict] = []
    for line in raw_lines:
        # Skip sub-info lines (indented) and empty lines
        if not line.strip() or line[0].isspace():
            continue
        parts = line.split()
        if len(parts) < 2:
            continue
        serial = parts[-1]
        device_name = " ".join(parts[:-1])
        # D435I ends with 'I'; D435 does not
        has_builtin_imu = device_name.split()[-1].upper().endswith("I")
        cameras.append({
            "serial": serial,
            "device_name": device_name,
            "has_builtin_imu": has_builtin_imu,
        })

    if len(cameras) > MAX_CAMERAS:
        print(f"[imu_test] WARNING: {len(cameras)} cameras found; using first {MAX_CAMERAS}")
        cameras = cameras[:MAX_CAMERAS]

    return cameras


def _rs_launch_args(ns: str, cam: dict) -> dict:
    """Build realsense2_camera launch arguments for one camera."""
    args = {
        "camera_namespace": ns,
        "camera_name": ns,
        "serial_no": f"_{cam['serial']}",
        "pointcloud.enable": "true",
        "align_depth.enable": "true",
        "depth_module.depth_profile": "640x480x30",
        "rgb_camera.color_profile": "640x480x30",
    }
    if cam["has_builtin_imu"]:
        args.update({
            "enable_gyro": "true",
            "enable_accel": "true",
            "unite_imu_method": "2",
            "gyro_fps": "200",
            "accel_fps": "200",
        })
    return args


def _setup_launch(context, *args, **kwargs):
    cameras = _detect_cameras()
    if not cameras:
        print("[imu_test] ERROR: No cameras detected. Nothing to launch.")
        return []

    print(f"[imu_test] Launching {len(cameras)} camera(s):")
    for i, cam in enumerate(cameras):
        ns = f"cam{i + 1}"
        print(f"  {ns}: {cam['device_name']}  serial={cam['serial']}"
              f"  has_builtin_imu={cam['has_builtin_imu']}")

    rs_launch_path = PathJoinSubstitution([
        FindPackageShare("realsense2_camera"), "launch", "rs_launch.py",
    ])

    actions = []
    camera_configs: list[dict] = []

    for i, cam in enumerate(cameras):
        ns = f"cam{i + 1}"
        launch_args = _rs_launch_args(ns, cam)
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rs_launch_path),
                launch_arguments=launch_args.items(),
            )
        )
        imu_topic = f"/{ns}/{ns}/imu" if cam["has_builtin_imu"] else f"/{ns}/data_raw"
        camera_configs.append({
            "name": ns,
            "serial": cam["serial"],
            "has_builtin_imu": cam["has_builtin_imu"],
            "imu_topic": imu_topic,
        })

    pkg_share = FindPackageShare("sensor_fusion_bringup").perform(context)
    config_dir = str(Path(pkg_share) / "config" / "openvins")

    dead_reckoning = Node(
        package="sensor_fusion_bringup",
        executable="imu_dead_reckoning_node.py",
        name="imu_dead_reckoning_node",
        output="screen",
        parameters=[{
            "camera_configs": json.dumps(camera_configs),
            "calibration_duration": 2.0,
            "config_dir": config_dir,
        }],
    )

    actions.append(TimerAction(period=5.0, actions=[dead_reckoning]))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        OpaqueFunction(function=_setup_launch),
    ])
