from datetime import datetime
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _arg_or_config(context, name: str, default):
    value = LaunchConfiguration(name).perform(context)
    return default if value == "" else value


def _mode_overrides(mode: str) -> dict:
    if mode == "observe":
        return {
            "dynamic_arm_measurement_only": True,
            "dynamic_arm_allow_initial_lock": False,
            "dynamic_arm_allow_reanchor": False,
            "dynamic_arm_reanchor_measurement_only": True,
        }
    if mode == "update":
        return {
            "dynamic_arm_measurement_only": False,
            "dynamic_arm_allow_initial_lock": False,
            "dynamic_arm_allow_reanchor": False,
            "dynamic_arm_reanchor_measurement_only": True,
        }
    if mode == "would_reanchor":
        return {
            "dynamic_arm_measurement_only": True,
            "dynamic_arm_allow_initial_lock": True,
            "dynamic_arm_allow_reanchor": True,
            "dynamic_arm_reanchor_measurement_only": True,
        }
    if mode == "active":
        return {
            "dynamic_arm_measurement_only": False,
            "dynamic_arm_allow_initial_lock": True,
            "dynamic_arm_allow_reanchor": True,
            "dynamic_arm_reanchor_measurement_only": False,
        }
    raise RuntimeError(f"Unsupported dynamic ID2 mode '{mode}'")


def _setup(context, *args, **kwargs):
    package_dir = Path(FindPackageShare("sensor_fusion_bringup").perform(context))
    config_path = Path(LaunchConfiguration("dynamic_config").perform(context))
    if not config_path.is_absolute():
        config_path = package_dir / config_path
    with open(config_path, "r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    launch_cfg = config.get("launch", {})
    measurement_cfg = config.get("measurement", {})
    openvins_cfg = config.get("openvins", {})
    pointcloud_cfg = config.get("pointcloud", {})

    mode = str(_arg_or_config(context, "mode", launch_cfg.get("mode", "observe"))).strip()
    start_cameras = _as_bool(_arg_or_config(context, "start_cameras", launch_cfg.get("start_cameras", True)))
    start_preview = _as_bool(_arg_or_config(context, "start_preview", launch_cfg.get("start_preview", True)))
    start_rviz = _as_bool(_arg_or_config(context, "start_rviz", launch_cfg.get("start_rviz", False)))
    record_bag = _as_bool(_arg_or_config(context, "record_bag", launch_cfg.get("record_bag", False)))
    enable_pointclouds = _as_bool(_arg_or_config(context, "enable_pointclouds", pointcloud_cfg.get("enable", False)))
    enable_marker_map_pointclouds = _as_bool(
        _arg_or_config(context, "enable_marker_map_pointclouds", pointcloud_cfg.get("enable_marker_map", enable_pointclouds))
    )
    pointcloud_decimation_enable = _as_bool(
        _arg_or_config(context, "pointcloud_decimation_enable", pointcloud_cfg.get("decimation_enable", enable_pointclouds))
    )
    record_pointclouds = _as_bool(_arg_or_config(context, "record_pointclouds", pointcloud_cfg.get("record", False)))
    use_sim_time = _as_bool(_arg_or_config(context, "use_sim_time", False))
    verbosity = str(_arg_or_config(context, "verbosity", launch_cfg.get("verbosity", "INFO")))
    hold_back_imu_for_frames = _as_bool(
        _arg_or_config(context, "hold_back_imu_for_frames", launch_cfg.get("hold_back_imu_for_frames", True))
    )
    marker_detection_rate_hz = str(_arg_or_config(context, "marker_detection_rate_hz", launch_cfg.get("marker_detection_rate_hz", 15.0)))
    pointcloud_max_rate_hz = str(_arg_or_config(context, "pointcloud_max_rate_hz", pointcloud_cfg.get("max_rate_hz", 15.0)))
    relay_pc_hz = str(_arg_or_config(context, "relay_pc_hz", launch_cfg.get("relay_pc_hz", pointcloud_max_rate_hz)))
    relay_pc_hz = launch_cfg.get("relay_pc_hz") or pointcloud_max_rate_hz
    pointcloud_voxel_leaf_m = str(_arg_or_config(context, "pointcloud_voxel_leaf_m", pointcloud_cfg.get("voxel_leaf_m", 0.01)))
    pointcloud_max_range_m = str(_arg_or_config(context, "pointcloud_max_range_m", pointcloud_cfg.get("max_range_m", 2.0)))
    pointcloud_decimation_magnitude = str(
        _arg_or_config(context, "pointcloud_decimation_magnitude", pointcloud_cfg.get("decimation_magnitude", 2))
    )
    pointcloud_require_marker_map_locked = str(
        _as_bool(_arg_or_config(context, "pointcloud_require_marker_map_locked", pointcloud_cfg.get("require_marker_map_locked", False)))
    ).lower()
    enable_pointcloud_neon_fix = str(
        _as_bool(_arg_or_config(context, "enable_pointcloud_neon_fix", pointcloud_cfg.get("enable_neon_fix", enable_pointclouds)))
    ).lower()

    dynamic_params = dict(openvins_cfg)
    dynamic_params.update(_mode_overrides(mode))
    dynamic_params["use_dynamic_arm_pose_updates"] = bool(dynamic_params.get("use_dynamic_arm_pose_updates", True))

    launch_dir = package_dir / "launch"
    rviz_config = package_dir / "config" / "rviz" / "phase2_dual_openvins_head_preview.rviz"
    head_marker_config = package_dir / "config" / "markers" / "head_aruco_map.yaml"
    arm_marker_config = package_dir / "config" / "markers" / "arm_aruco_map.yaml"
    arm_marker_extrinsics = package_dir / "config" / "markers" / "arm_marker_extrinsics.yaml"

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "head_d435i_openvins_phase2.launch.py")),
            launch_arguments={
                "start_camera": str(start_cameras).lower(),
                "use_sim_time": str(use_sim_time).lower(),
                "verbosity": verbosity,
                "hold_back_imu_for_frames": str(hold_back_imu_for_frames).lower(),
                "enable_pointclouds": str(enable_pointclouds).lower(),
                "enable_marker_map_pointclouds": str(enable_marker_map_pointclouds).lower(),
                "pointcloud_decimation_enable": str(pointcloud_decimation_enable).lower(),
                "pointcloud_max_rate_hz": pointcloud_max_rate_hz,
                "pointcloud_voxel_leaf_m": pointcloud_voxel_leaf_m,
                "pointcloud_max_range_m": pointcloud_max_range_m,
                "pointcloud_decimation_magnitude": pointcloud_decimation_magnitude,
                "pointcloud_require_marker_map_locked": pointcloud_require_marker_map_locked,
                "enable_pointcloud_neon_fix": enable_pointcloud_neon_fix,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "arm_d435i_openvins_phase2.launch.py")),
            launch_arguments={
                "start_camera": str(start_cameras).lower(),
                "use_sim_time": str(use_sim_time).lower(),
                "verbosity": verbosity,
                "hold_back_imu_for_frames": str(hold_back_imu_for_frames).lower(),
                "enable_pointclouds": str(enable_pointclouds).lower(),
                "enable_marker_map_pointclouds": str(enable_marker_map_pointclouds).lower(),
                "pointcloud_decimation_enable": str(pointcloud_decimation_enable).lower(),
                "pointcloud_max_rate_hz": pointcloud_max_rate_hz,
                "pointcloud_voxel_leaf_m": pointcloud_voxel_leaf_m,
                "pointcloud_max_range_m": pointcloud_max_range_m,
                "pointcloud_decimation_magnitude": pointcloud_decimation_magnitude,
                "pointcloud_require_marker_map_locked": pointcloud_require_marker_map_locked,
                "enable_pointcloud_neon_fix": enable_pointcloud_neon_fix,
                "use_dynamic_arm_pose_updates": str(dynamic_params["use_dynamic_arm_pose_updates"]).lower(),
                "dynamic_arm_measurement_only": str(dynamic_params["dynamic_arm_measurement_only"]).lower(),
                "dynamic_arm_pose_topic": str(dynamic_params.get("dynamic_arm_pose_topic", "/arm/marker_pose/dynamic_arm_pose_observation")),
                "dynamic_arm_status_topic": str(dynamic_params.get("dynamic_arm_status_topic", "/ov_msckf_arm/dynamic_arm_update/status")),
                "dynamic_arm_global_frame_id": str(dynamic_params.get("dynamic_arm_global_frame_id", "marker_map")),
                "dynamic_arm_target_frame": str(dynamic_params.get("dynamic_arm_target_frame", "arm_imu")),
                "dynamic_arm_source_camera_frame": str(
                    dynamic_params.get("dynamic_arm_source_camera_frame", "head_d435i_head_color_optical_frame")
                ),
                "dynamic_arm_marker_frame": str(dynamic_params.get("dynamic_arm_marker_frame", "arm_marker_2")),
                "dynamic_arm_marker_id": str(dynamic_params.get("dynamic_arm_marker_id", 2)),
                "dynamic_arm_time_tolerance_s": str(dynamic_params.get("dynamic_arm_time_tolerance_s", 0.05)),
                "dynamic_arm_noise_multiplier": str(dynamic_params.get("dynamic_arm_noise_multiplier", 4.0)),
                "dynamic_arm_chi2_gate": str(dynamic_params.get("dynamic_arm_chi2_gate", 16.81)),
                "dynamic_arm_max_update_translation_m": str(dynamic_params.get("dynamic_arm_max_update_translation_m", 0.35)),
                "dynamic_arm_max_update_rotation_deg": str(dynamic_params.get("dynamic_arm_max_update_rotation_deg", 15.0)),
                "dynamic_arm_min_update_interval_s": str(dynamic_params.get("dynamic_arm_min_update_interval_s", 0.10)),
                "dynamic_arm_skip_after_fixed_marker_s": str(dynamic_params.get("dynamic_arm_skip_after_fixed_marker_s", 0.50)),
                "dynamic_arm_allow_initial_lock": str(dynamic_params["dynamic_arm_allow_initial_lock"]).lower(),
                "dynamic_arm_allow_reanchor": str(dynamic_params["dynamic_arm_allow_reanchor"]).lower(),
                "dynamic_arm_reanchor_measurement_only": str(dynamic_params["dynamic_arm_reanchor_measurement_only"]).lower(),
                "dynamic_arm_reanchor_min_samples": str(dynamic_params.get("dynamic_arm_reanchor_min_samples", 5)),
                "dynamic_arm_reanchor_window_s": str(dynamic_params.get("dynamic_arm_reanchor_window_s", 2.0)),
                "dynamic_arm_reanchor_min_sample_dt_s": str(dynamic_params.get("dynamic_arm_reanchor_min_sample_dt_s", 0.50)),
                "dynamic_arm_reanchor_max_velocity_mps": str(dynamic_params.get("dynamic_arm_reanchor_max_velocity_mps", 2.0)),
                "dynamic_arm_reanchor_max_sample_translation_std_m": str(
                    dynamic_params.get("dynamic_arm_reanchor_max_sample_translation_std_m", 0.12)
                ),
                "dynamic_arm_reanchor_max_sample_rotation_std_deg": str(
                    dynamic_params.get("dynamic_arm_reanchor_max_sample_rotation_std_deg", 8.0)
                ),
                "dynamic_arm_reanchor_trigger_translation_m": str(dynamic_params.get("dynamic_arm_reanchor_trigger_translation_m", 0.75)),
                "dynamic_arm_reanchor_trigger_rotation_deg": str(dynamic_params.get("dynamic_arm_reanchor_trigger_rotation_deg", 20.0)),
                "dynamic_arm_reanchor_cooldown_s": str(dynamic_params.get("dynamic_arm_reanchor_cooldown_s", 5.0)),
                "dynamic_arm_reanchor_skip_after_fixed_marker_s": str(
                    dynamic_params.get("dynamic_arm_reanchor_skip_after_fixed_marker_s", 3.0)
                ),
                "dynamic_arm_reanchor_covariance_multiplier": str(dynamic_params.get("dynamic_arm_reanchor_covariance_multiplier", 2.0)),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "head_marker_pose_phase2.launch.py")),
            launch_arguments={
                "config_file": str(head_marker_config),
                "use_sim_time": str(use_sim_time).lower(),
                "marker_detection_rate_hz": marker_detection_rate_hz,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "arm_marker_pose_phase2.launch.py")),
            launch_arguments={
                "config_file": str(arm_marker_config),
                "use_sim_time": str(use_sim_time).lower(),
                "marker_detection_rate_hz": marker_detection_rate_hz,
            }.items(),
        ),
        Node(
            package="sensor_fusion_bringup",
            executable="dynamic_arm_pose_measurement_node.py",
            name="dynamic_arm_pose_measurement_node",
            output="screen",
            parameters=[
                {"use_sim_time": use_sim_time},
                {"head_marker_config": str(head_marker_config)},
                {"arm_marker_config": str(arm_marker_config)},
                {"arm_marker_extrinsics": str(arm_marker_extrinsics)},
                {"dynamic_observation_topic": str(measurement_cfg.get("dynamic_observation_topic", "/head/marker_pose/dynamic_observation"))},
                {"head_pose_topic": str(measurement_cfg.get("head_pose_topic", "/ov_msckf/odomimu"))},
                {"head_pose_message_type": str(measurement_cfg.get("head_pose_message_type", "odometry"))},
                {
                    "dynamic_arm_pose_observation_topic": str(
                        measurement_cfg.get("dynamic_arm_pose_observation_topic", "/arm/marker_pose/dynamic_arm_pose_observation")
                    )
                },
                {
                    "dynamic_arm_measurement_status_topic": str(
                        measurement_cfg.get("dynamic_arm_measurement_status_topic", "/arm/marker_pose/dynamic_arm_measurement/status")
                    )
                },
                {
                    "publish_dynamic_arm_pose_observation": bool(
                        measurement_cfg.get("publish_dynamic_arm_pose_observation", True)
                    )
                },
                {"marker_id": int(measurement_cfg.get("marker_id", 2))},
                {"target_frame": str(measurement_cfg.get("target_frame", "arm_imu"))},
                {"max_head_pose_dt_s": float(measurement_cfg.get("max_head_pose_dt_s", 0.05))},
                {"head_pose_buffer_seconds": float(measurement_cfg.get("head_pose_buffer_seconds", 5.0))},
                {"require_stable_dynamic_marker": bool(measurement_cfg.get("require_stable_dynamic_marker", True))},
                {"max_pose_covariance_trace": float(measurement_cfg.get("max_pose_covariance_trace", 10.0))},
                {"max_reprojection_error_px": float(measurement_cfg.get("max_reprojection_error_px", 3.0))},
                {"max_marker_distance_m": float(measurement_cfg.get("max_marker_distance_m", 2.0))},
                {"max_view_angle_deg": float(measurement_cfg.get("max_view_angle_deg", 75.0))},
                {"min_marker_area_px2": float(measurement_cfg.get("min_marker_area_px2", 800.0))},
                {"min_geometry_score": float(measurement_cfg.get("min_geometry_score", 0.35))},
                {"extrinsic_covariance_source": str(measurement_cfg.get("extrinsic_covariance_source", "robust_diag_covariance_se3"))},
            ],
        ),
    ]

    if start_preview:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(launch_dir / "head_derived_arm_pose_preview.launch.py")),
                launch_arguments={
                    "use_sim_time": str(use_sim_time).lower(),
                    "require_stable_dynamic_marker": str(measurement_cfg.get("require_stable_dynamic_marker", True)).lower(),
                    "publish_tf": "true",
                }.items(),
            )
        )

    if start_rviz:
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2", arguments=["-d", str(rviz_config)], output="screen"))

    if record_bag:
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        bag_path = f"bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_{stamp}"
        topics = [
            "/tf",
            "/tf_static",
            "/rosout",
            "/head/d435i_head/color/image_raw",
            "/head/d435i_head/color/camera_info",
            "/head/d435i_head/imu",
            "/arm/d435i_arm/color/image_raw",
            "/arm/d435i_arm/color/camera_info",
            "/arm/d435i_arm/imu",
            "/head/marker_pose/observation",
            "/arm/marker_pose/observation",
            "/head/marker_pose/dynamic_observation",
            "/arm/marker_pose/dynamic_arm_pose_observation",
            "/arm/marker_pose/dynamic_arm_measurement/status",
            "/ov_msckf_arm/dynamic_arm_update/status",
            "/ov_msckf/poseimu",
            "/ov_msckf/odomimu",
            "/ov_msckf/pathimu",
            "/ov_msckf/marker_map_locked",
            "/ov_msckf_arm/poseimu",
            "/ov_msckf_arm/odomimu",
            "/ov_msckf_arm/pathimu",
            "/ov_msckf_arm/marker_map_locked",
            "/arm/marker_pose/head_derived/arm_camera_pose",
            "/arm/marker_pose/head_derived/path",
        ]
        if record_pointclouds:
            topics.extend(
                [
                    "/head/d435i_head/depth/color/points",
                    "/arm/d435i_arm/depth/color/points",
                ]
            )
            if enable_marker_map_pointclouds:
                topics.extend(
                    [
                        "/head/d435i_head/points_marker_map",
                        "/head/d435i_head/points_marker_map/status",
                        "/arm/d435i_arm/points_marker_map",
                        "/arm/d435i_arm/points_marker_map/status",
                    ]
                )
        actions.append(
            ExecuteProcess(
                cmd=["bash", "-lc", "mkdir -p bags/openvins_tests/phase2_live && ros2 bag record -o " + bag_path + " " + " ".join(topics)],
                output="screen",
            )
        )

    # ── Jetson → Laptop relay ─────────────────────────────────────────
    jetson_relay_enabled = _as_bool(
        _arg_or_config(context, "jetson_relay_enabled", True)
    )
    if jetson_relay_enabled:
        relay_pc_decimate = str(
            _as_bool(_arg_or_config(context, "relay_pc_decimate", True))
        ).lower()

        # relay_hz is a master override: when set (non-empty), it applies to
        # pointclouds, images, AND trackhist simultaneously.  Individual
        # relay_*_hz args still take precedence when explicitly set.
        relay_hz_raw = str(_arg_or_config(context, "relay_hz", "")).strip()

        pc_default = relay_hz_raw if relay_hz_raw else pointcloud_max_rate_hz
        img_default = relay_hz_raw if relay_hz_raw else "5.0"
        trackhist_default = relay_hz_raw if relay_hz_raw else img_default

        relay_pc_hz = str(
            _arg_or_config(context, "relay_pc_hz", pc_default)
        )
        relay_img_hz = str(
            _arg_or_config(context, "relay_img_hz", img_default)
        )
        relay_trackhist_hz = str(
            _arg_or_config(context, "relay_trackhist_hz", trackhist_default)
        )
        # Use the bind-mounted source file directly — avoids dependency on
        # overlay rebuild (scripts/ is not in the share install directory).
        relay_script = "/miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/scripts/jetson_relay.py"
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3", relay_script,
                    "--ros-args",
                    "-p", f"pointcloud.hz:={relay_pc_hz}",
                    "-p", f"pointcloud.decimation.enabled:={relay_pc_decimate}",
                    "-p", f"image.hz:={relay_img_hz}",
                    "-p", f"image.downsample_factor:={str(_arg_or_config(context, 'image.downsample_factor', 1))}",
                    "-p", f"trackhist.hz:={relay_trackhist_hz}",
                    # Depth channel: NEAREST 2× downscale + throttle for
                    # host-side depth_image_proc backprojection.  Must use
                    # aligned_depth_to_color so depth and colour pixels map
                    # 1:1 without an additional frame transform.
                    "-p", "depth.enabled:=true",
                    "-p", f"depth.hz:={relay_img_hz}",
                    "-p", "depth.downsample_factor:=2",
                    # Enable ArUco relay so marker observations flow to the
                    # host GTSAM tracker as prior/between factors.
                    "-p", "aruco.enabled:=true",
                ],
                name="jetson_relay",
                output="screen",
            )
        )

    # ── TF throttle ────────────────────────────────────────────────────
    # Caps /tf broadcast per child frame to prevent DDS retransmission
    # storms.  The two aruco nodes publish corrected odom TFs at ~200 Hz
    # each; without throttling the combined /tf rate approaches 1.5 kHz
    # which starves PointCloud2 streams of network bandwidth.
    tf_throttle_enabled = _as_bool(
        _arg_or_config(context, "tf_throttle_enabled", True)
    )
    if tf_throttle_enabled:
        tf_throttle_hz = str(
            _arg_or_config(context, "tf_throttle_hz", "50.0")
        )
        tf_throttle_frames = str(
            _arg_or_config(
                context, "tf_throttle_frames",
                '["head_imu_openvins_corrected","arm_imu_openvins_corrected","head_imu","arm_imu"]'
            )
        )
        throttle_script = ("/miahand_ws/src/multi_cam_localization/"
                           "sensor_fusion_bringup/scripts/tf_throttle_node.py")
        actions.append(
            ExecuteProcess(
                cmd=[
                    "python3", throttle_script,
                    "--ros-args",
                    "-p", f"max_hz:={tf_throttle_hz}",
                    "-p", f"subscribe_topic:=/tf_raw",
                    "-p", f"child_frames:={tf_throttle_frames}",
                ],
                name="tf_throttle",
                output="screen",
            )
        )
    return actions


def generate_launch_description():
    default_config = "config/dynamic_id2_arm_update.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("dynamic_config", default_value=default_config),
            DeclareLaunchArgument(
                "mode",
                default_value="",
                description="Workflow mode. Empty uses launch.mode from dynamic_config, currently active.",
            ),
            DeclareLaunchArgument("start_cameras", default_value=""),
            DeclareLaunchArgument("start_preview", default_value=""),
            DeclareLaunchArgument("start_rviz", default_value=""),
            DeclareLaunchArgument("record_bag", default_value=""),
            DeclareLaunchArgument("enable_pointclouds", default_value=""),
            DeclareLaunchArgument("enable_marker_map_pointclouds", default_value=""),
            DeclareLaunchArgument("pointcloud_decimation_enable", default_value=""),
            DeclareLaunchArgument("pointcloud_max_rate_hz", default_value=""),
            DeclareLaunchArgument("pointcloud_voxel_leaf_m", default_value=""),
            DeclareLaunchArgument("pointcloud_max_range_m", default_value=""),
            DeclareLaunchArgument("pointcloud_decimation_magnitude", default_value=""),
            DeclareLaunchArgument("pointcloud_require_marker_map_locked", default_value=""),
            DeclareLaunchArgument(
                "image.downsample_factor",
                default_value="",
                description="Color image downsample factor passed to the relay (empty = relay code default of 1).",
            ),
            DeclareLaunchArgument(
                "marker_detection_rate_hz",
                default_value="",
                description="ArUco marker detection throttle Hz (empty = use launch.marker_detection_rate_hz from dynamic_config).",
            ),
            DeclareLaunchArgument("enable_pointcloud_neon_fix", default_value=""),
            DeclareLaunchArgument("record_pointclouds", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("verbosity", default_value=""),
            DeclareLaunchArgument("hold_back_imu_for_frames", default_value=""),
            DeclareLaunchArgument(
                "jetson_relay_enabled",
                default_value="true",
                description="Run the jetson_relay node to throttle/compress sensor data for the laptop.",
            ),
            DeclareLaunchArgument(
                "relay_hz",
                default_value="",
                description="Master Hz override for ALL relay streams (pc + image + trackhist). "
                            "Empty = use individual relay_*_hz args.",
            ),
            DeclareLaunchArgument(
                "relay_pc_decimate",
                default_value="true",
                description="Enable stride decimation on relayed pointclouds.",
            ),
            DeclareLaunchArgument(
                "relay_pc_hz",
                default_value="",
                description="Override pointcloud relay throttle Hz (empty = use pointcloud_max_rate_hz).",
            ),
            DeclareLaunchArgument(
                "relay_img_hz",
                default_value="5.0",
                description="Image relay throttle frequency in Hz.",
            ),
            DeclareLaunchArgument(
                "relay_trackhist_hz",
                default_value="",
                description="Trackhist relay throttle Hz (empty = use relay_img_hz).",
            ),
            DeclareLaunchArgument(
                "tf_throttle_enabled",
                default_value="true",
                description="Enable /tf throttle to cap broadcast rate per child frame "
                            "and prevent DDS retransmission storms.",
            ),
            DeclareLaunchArgument(
                "tf_throttle_hz",
                default_value="50.0",
                description="Maximum TF broadcast rate per child frame (Hz).",
            ),
            DeclareLaunchArgument(
                "tf_throttle_frames",
                default_value='["head_imu_openvins_corrected","arm_imu_openvins_corrected","head_imu","arm_imu"]',
                description="JSON list of child frame IDs to throttle on /tf.",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
