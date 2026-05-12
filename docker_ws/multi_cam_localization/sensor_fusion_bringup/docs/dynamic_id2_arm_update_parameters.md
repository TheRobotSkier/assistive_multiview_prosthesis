# Dynamic ID2 Arm Update Parameters

User-editable config:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml
```

Dynamic ID2 is separate from the fixed ID0 marker-map path. Keep head and arm
`marker_fixed_ids` at `"0"`.

## Launch

| Parameter | Default | Description |
| --- | ---: | --- |
| `mode` | `observe` | Workflow mode: `observe`, `update`, `would_reanchor`, or `active`. |
| `start_cameras` | `true` | Start RealSense camera drivers through the head/arm OpenVINS launches. |
| `start_preview` | `true` | Start the debug RViz preview path. Preview is not used by OpenVINS updates. |
| `start_rviz` | `false` | Start RViz with the phase2 preview config. |
| `record_bag` | `false` | Record the standard validation topics from the single launch. |
| `marker_detection_rate_hz` | `15.0` | Marker detector throttle for head and arm marker nodes. |
| `verbosity` | `INFO` | OpenVINS verbosity. |

Mode mapping:

| Mode | Normal Updates | Initial-Lock/Reanchor | State Mutation |
| --- | --- | --- | --- |
| `observe` | measurement-only | disabled | no dynamic mutation |
| `update` | enabled | disabled | only bounded EKF updates |
| `would_reanchor` | measurement-only | measurement-only | no dynamic mutation |
| `active` | enabled | enabled | bounded updates and guarded reanchor |

## Measurement

| Parameter | Default | Units | Description |
| --- | ---: | --- | --- |
| `publish_dynamic_arm_pose_observation` | `true` | bool | Publish `/arm/marker_pose/dynamic_arm_pose_observation`. |
| `dynamic_observation_topic` | `/head/marker_pose/dynamic_observation` | topic | Input ID2 observations from the head camera. |
| `head_pose_topic` | `/ov_msckf/odomimu` | topic | High-rate head pose source. Prefer odometry over low-rate pose. |
| `head_pose_message_type` | `odometry` | enum | `odometry` or `pose_with_covariance_stamped`. Odometry must be `marker_map -> head_imu`. |
| `dynamic_arm_pose_observation_topic` | `/arm/marker_pose/dynamic_arm_pose_observation` | topic | Output covariance-bearing arm IMU pose measurement. |
| `dynamic_arm_measurement_status_topic` | `/arm/marker_pose/dynamic_arm_measurement/status` | topic | JSON status from the measurement node. |
| `marker_id` | `2` | id | Dynamic marker id mounted on the arm. |
| `target_frame` | `arm_imu` | frame | Target arm frame whose pose is measured. |
| `max_head_pose_dt_s` | `0.05` | s | Maximum allowed timestamp difference. No pending ID2 queue is used. |
| `head_pose_buffer_seconds` | `5.0` | s | Retained head pose history used for already-buffered matching. |
| `require_stable_dynamic_marker` | `true` | bool | Require marker temporal stability before producing a measurement. |
| `max_pose_covariance_trace` | `10.0` | mixed | Reject head poses with very large covariance trace. |
| `max_reprojection_error_px` | `3.0` | px | Reject poor ID2 solvePnP quality. |
| `max_marker_distance_m` | `2.0` | m | Reject ID2 detections too far from the head camera. |
| `max_view_angle_deg` | `75.0` | deg | Reject very oblique ID2 views. |
| `min_marker_area_px2` | `800.0` | px^2 | Reject tiny marker detections. |
| `min_geometry_score` | `0.35` | unitless | Reject poor corner geometry. |
| `extrinsic_covariance_source` | `robust_diag_covariance_se3` | enum | Covariance source from `arm_marker_extrinsics.yaml`. |

Safe tuning:

- Keep `max_head_pose_dt_s` at `0.05` unless status proves timing is the only
  blocker and accepted offsets remain small.
- If measurement rate is low, inspect status reasons before relaxing marker
  quality gates.
- Do not add a delayed pending-observation queue for EKF updates.

## OpenVINS Normal Dynamic Update

| Parameter | Default | Units | Description |
| --- | ---: | --- | --- |
| `use_dynamic_arm_pose_updates` | `true` | bool | Enable the arm OpenVINS dynamic subscriber in the single launch. |
| `dynamic_arm_measurement_only` | `true` | bool | Gate/log dynamic measurements without mutating state. |
| `dynamic_arm_pose_topic` | `/arm/marker_pose/dynamic_arm_pose_observation` | topic | Dynamic arm measurement input to arm OpenVINS. |
| `dynamic_arm_status_topic` | `/ov_msckf_arm/dynamic_arm_update/status` | topic | JSON status from the OpenVINS dynamic updater. |
| `dynamic_arm_global_frame_id` | `marker_map` | frame | Required global frame of dynamic arm measurements. |
| `dynamic_arm_target_frame` | `arm_imu` | frame | Required target frame; the updater constrains the arm IMU state. |
| `dynamic_arm_source_camera_frame` | `head_d435i_head_color_optical_frame` | frame | Required source camera frame for ID2 observations. |
| `dynamic_arm_marker_frame` | `arm_marker_2` | frame | Required dynamic marker frame. |
| `dynamic_arm_marker_id` | `2` | id | Required dynamic marker id. |
| `dynamic_arm_time_tolerance_s` | `0.05` | s | Maximum difference between measurement stamp and arm OpenVINS state time. |
| `dynamic_arm_noise_multiplier` | `4.0` | scale | Inflates measurement covariance for conservative updates. |
| `dynamic_arm_chi2_gate` | `16.81` | chi2 | Innovation gate for the 6D residual. |
| `dynamic_arm_max_update_translation_m` | `0.35` | m | Reject normal EKF updates with larger translation innovation. |
| `dynamic_arm_max_update_rotation_deg` | `15.0` | deg | Reject normal EKF updates with larger rotation innovation. |
| `dynamic_arm_min_update_interval_s` | `0.10` | s | Minimum time between accepted normal dynamic updates. |
| `dynamic_arm_skip_after_fixed_marker_s` | `0.50` | s | Skip dynamic updates shortly after accepted fixed ID0 updates. |

Normal dynamic updates are for small, covariance-bearing corrections only. Large
post-gap disagreements should be handled by `would_reanchor`/`active`, not by
loosening the normal EKF gate first.

## Dynamic Initial-Lock/Reanchor

| Parameter | Default | Units | Description |
| --- | ---: | --- | --- |
| `dynamic_arm_allow_initial_lock` | `false` | bool | Allow dynamic ID2 to perform first arm `marker_map` lock. |
| `dynamic_arm_allow_reanchor` | `false` | bool | Allow dynamic ID2 to reanchor after large rejected innovations. |
| `dynamic_arm_reanchor_measurement_only` | `true` | bool | Report `would_dynamic_*` without mutating state. |
| `dynamic_arm_reanchor_min_samples` | `5` | count | Required recent dynamic samples. |
| `dynamic_arm_reanchor_window_s` | `2.0` | s | Recent sample window used for velocity/scatter checks. |
| `dynamic_arm_reanchor_min_sample_dt_s` | `0.50` | s | Required time span between oldest and newest usable samples. |
| `dynamic_arm_reanchor_max_velocity_mps` | `2.0` | m/s | Reject implausibly fast dynamic reanchor fits. |
| `dynamic_arm_reanchor_max_sample_translation_std_m` | `0.12` | m | Reject inconsistent recent translation samples. |
| `dynamic_arm_reanchor_max_sample_rotation_std_deg` | `8.0` | deg | Reject inconsistent recent orientation samples. |
| `dynamic_arm_reanchor_trigger_translation_m` | `0.75` | m | Minimum large innovation for reanchor evaluation when already locked. |
| `dynamic_arm_reanchor_trigger_rotation_deg` | `20.0` | deg | Minimum large rotation innovation for reanchor evaluation when already locked. |
| `dynamic_arm_reanchor_cooldown_s` | `5.0` | s | Prevent repeated dynamic reanchors. |
| `dynamic_arm_reanchor_skip_after_fixed_marker_s` | `3.0` | s | Fixed ID0 has priority over dynamic reanchor. |
| `dynamic_arm_reanchor_covariance_multiplier` | `2.0` | scale | Extra covariance inflation for dynamic reset/reanchor. |

Enable sequence:

1. `observe`: verify measurement status and covariance.
2. `update`: verify small normal updates improve behavior without snaps.
3. `would_reanchor`: verify candidates, sample count, velocity, scatter, cooldown,
   and fixed-ID0 skip state.
4. `active`: enable only after replay and live would-reanchor are explainable.

## Acceptance Checklist

- ID2 does not appear in `marker_fixed_ids`.
- No delayed queued ID2 measurements are applied to the EKF.
- Status explains accepted, rejected, skipped, and would-reanchor events.
- Fixed ID0 still locks/reanchors when visible and has priority over dynamic ID2.
- Dynamic reanchor is explicit, guarded, and default-disabled outside the single
  launch mode mapping.
