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
    openvins_experiment_profile = str(
        _arg_or_config(context, "openvins_experiment_profile", "baseline")
    ).strip()
    start_cameras = _as_bool(_arg_or_config(context, "start_cameras", launch_cfg.get("start_cameras", True)))
    start_preview = _as_bool(_arg_or_config(context, "start_preview", launch_cfg.get("start_preview", True)))
    start_rviz = _as_bool(_arg_or_config(context, "start_rviz", launch_cfg.get("start_rviz", False)))
    start_marker_graph = _as_bool(_arg_or_config(context, "start_marker_graph", launch_cfg.get("start_marker_graph", True)))
    start_marker_graph_odom_relay = _as_bool(_arg_or_config(context, "start_marker_graph_odom_relay", launch_cfg.get("start_marker_graph_odom_relay", True)))
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
    marker_detection_rate_hz = str(launch_cfg.get("marker_detection_rate_hz", 15.0))
    pointcloud_max_rate_hz = str(_arg_or_config(context, "pointcloud_max_rate_hz", pointcloud_cfg.get("max_rate_hz", 15.0)))
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
                "openvins_experiment_profile": openvins_experiment_profile,
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
                "openvins_experiment_profile": openvins_experiment_profile,
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

    marker_graph_cfg = config.get("marker_graph", {})
    if start_marker_graph:
        actions.append(
            Node(
                package="sensor_fusion_bringup",
                executable="marker_graph_estimator.py",
                name="marker_graph_estimator",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"head_observation_topic": str(marker_graph_cfg.get("head_observation_topic", "/head/marker_pose/observation"))},
                    {"arm_observation_topic": str(marker_graph_cfg.get("arm_observation_topic", "/arm/marker_pose/observation"))},
                    {"max_edge_age_s": float(marker_graph_cfg.get("max_edge_age_s", 30.0))},
                    {"coobservation_time_window_s": float(marker_graph_cfg.get("coobservation_time_window_s", 0.05))},
                    {"min_edge_quality": float(marker_graph_cfg.get("min_edge_quality", 0.2))},
                    {"max_hops": int(marker_graph_cfg.get("max_hops", 10))},
                    {"max_observation_age_s": float(marker_graph_cfg.get("max_observation_age_s", 5.0))},
                    {"covariance_growth_xyz_m": float(marker_graph_cfg.get("covariance_growth_xyz_m", 0.01))},
                    {"covariance_growth_rpy_rad": float(marker_graph_cfg.get("covariance_growth_rpy_rad", 0.0174533))},
                    {"head_source_name": str(marker_graph_cfg.get("head_source_name", "head"))},
                    {"arm_source_name": str(marker_graph_cfg.get("arm_source_name", "arm"))},
                    {"head_imu_frame": str(marker_graph_cfg.get("head_imu_frame", "head_imu"))},
                    {"arm_imu_frame": str(marker_graph_cfg.get("arm_imu_frame", "arm_imu"))},
                    {"map_frame": str(marker_graph_cfg.get("map_frame", "marker_map"))},
                    {"publish_rate_hz": float(marker_graph_cfg.get("publish_rate_hz", 10.0))},
                    {"observation_buffer_s": float(marker_graph_cfg.get("observation_buffer_s", 0.5))},
                ],
            )
        )

    odom_relay_cfg = config.get("marker_graph_odom_relay", {})
    if start_marker_graph_odom_relay:
        actions.append(
            Node(
                package="sensor_fusion_bringup",
                executable="marker_graph_odom_relay.py",
                name="marker_graph_odom_relay",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"head_to_arm_topic": str(odom_relay_cfg.get("head_to_arm_topic", "/marker_graph_estimator/head_to_arm"))},
                    {"graph_status_topic": str(odom_relay_cfg.get("graph_status_topic", "/marker_graph_estimator/status"))},
                    {"head_odom_topic": str(odom_relay_cfg.get("head_odom_topic", "/ov_msckf/odomimu"))},
                    {"arm_odom_topic": str(odom_relay_cfg.get("arm_odom_topic", "/ov_msckf_arm/odomimu"))},
                    {"corrected_odom_topic": str(odom_relay_cfg.get("corrected_odom_topic", "/ov_msckf_arm/odomimu_corrected"))},
                    {"correction_status_topic": str(odom_relay_cfg.get("correction_status_topic", "/marker_graph/correction_status"))},
                    {"corrected_child_frame_id": str(odom_relay_cfg.get("corrected_child_frame_id", "arm_imu_corrected"))},
                    {"min_graph_chain_quality": float(odom_relay_cfg.get("min_graph_chain_quality", 0.2))},
                    {"publish_rate_hz": float(odom_relay_cfg.get("publish_rate_hz", 10.0))},
                    {"max_head_odom_age_s": float(odom_relay_cfg.get("max_head_odom_age_s", 0.5))},
                    {"max_arm_odom_age_s": float(odom_relay_cfg.get("max_arm_odom_age_s", 0.5))},
                    {"max_graph_age_s": float(odom_relay_cfg.get("max_graph_age_s", 1.0))},
                ],
            )
        )

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
            "/ov_msckf_arm/odomimu_corrected",
            "/marker_graph/correction_status",
            "/marker_graph_estimator/head_to_arm",
            "/marker_graph_estimator/status",
            "/marker_graph_estimator/edges",
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
            DeclareLaunchArgument("enable_pointcloud_neon_fix", default_value=""),
            DeclareLaunchArgument("record_pointclouds", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("verbosity", default_value=""),
            DeclareLaunchArgument("hold_back_imu_for_frames", default_value=""),
            DeclareLaunchArgument("start_marker_graph_odom_relay", default_value=""),
            DeclareLaunchArgument(
                "openvins_experiment_profile",
                default_value="",
                description=(
                    "Named experiment profile forwarded to head/arm OpenVINS launches. "
                    "Empty/unset = baseline (unchanged defaults). "
                    "See config/openvins_experiment_profiles.yaml for available profiles."
                ),
            ),
            OpaqueFunction(function=_setup),
        ]
    )
