# Dynamic ID2 Arm OpenVINS Update Planning Prompt

Use this prompt in this chat or a fresh Codex chat when ready to plan the next
implementation. Start in plan mode and do not code yet.

```text
Read:
- .agents/AGENTS.md
- .agents/future_tasks.md
- .agents/phase2_openvins_handoff_status.md
- .agents/dynamic_id2_openvins_update_plan_prompt.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/head_derived_arm_pose_preview_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/head_derived_arm_pose_preview_node.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
- docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
- docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/DynamicMarkerObservation.msg
- docker_ws/multi_cam_localization/sensor_fusion_msgs/msg/MarkerPoseObservation.msg
- docker_ws/src/open_vins/ov_msckf

Plan mode only. Do not change code.

Make a detailed implementation plan for adding dynamic marker ID2 updates of
the arm D435i state from the head D435i into the modified OpenVINS VIO +
static-marker update code.

Important boundaries:
- Do not add ID2 to `marker_fixed_ids`.
- Keep dynamic ID2 separate from the fixed marker-map landmark update path.
- Do not treat the head-derived arm pose as ground truth.
- Preserve the existing fixed marker ID0 update/reanchor behavior.
- Keep head/arm OpenVINS frames separated: `head_imu`, `head_cam0`,
  `arm_imu`, `arm_cam0`.

Use these validation artifacts:
- Calibration-source bag:
  `docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816`
- Older runtime dynamic-ID2 full bag:
  `docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_20260511_194110`
- Debug preview validation bag:
  `docker_ws/bags/openvins_tests/phase2_live/head_derived_arm_preview_20260512_073918`
- Preferred full raw dynamic-ID2 update planning bag:
  `docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_full_raw_20260512_090704`

Known preview validation result:
- `head_derived_arm_preview_20260512_073918` contains `162` dynamic ID2
  observations and `48` accepted preview poses.
- Dynamic marker quality was good: median reprojection `0.252 px`, p95
  reprojection `0.535 px`, median distance `0.582 m`, p95 view angle
  `16.95 deg`.
- Preview vs arm OpenVINS camera-frame comparison: `42` matched samples,
  median translation residual `0.0646 m`, p95 translation residual `0.1978 m`,
  median rotation residual `2.06 deg`, p95 rotation residual `3.69 deg`.
- The preview bag is not a complete raw replay source because it did not record
  raw camera/IMU streams or fixed marker observation topics.

Known full raw planning-bag validation result:
- `dynamic_id2_arm_update_full_raw_20260512_090704` contains both head/arm raw
  color image, camera-info, and IMU streams; fixed marker observations; dynamic
  ID2 observations; current OpenVINS outputs; preview outputs; `/tf`; and
  `/tf_static`.
- Bag metadata: `62.39 s`, `2.9 GiB`, `71417` messages.
- Fixed marker observations are ID0-only: `525` head observations targeting
  `head_imu`, `302` arm observations targeting `arm_imu`.
- Dynamic observations are ID2-only: `111` messages in
  `head_d435i_head_color_optical_frame`, with `104` stable observations.
- Dynamic marker quality: median reprojection `0.274 px`, p95 reprojection
  `0.604 px`, median distance `0.528 m`, p95 view angle `22.00 deg`.
- Preview output: `18` accepted head-derived arm-camera poses in `marker_map`,
  finite covariance, `_head_preview` TF frames present.
- Offline raw-image calibration characterization from this bag: head ID0
  `1578`, head ID2 `373`, head ID0+ID2 `373`, arm ID0 `822`, `159`
  synchronized calibration samples, `154` inliers, p95 translation residual
  `0.0120 m`, p95 rotation residual `3.839 deg`.
- Preview vs current arm OpenVINS converted to arm camera frame had large
  translation disagreement: `17` matched samples, median translation residual
  `1.023 m`, p95 translation residual `22.11 m`, median rotation residual
  `2.97 deg`, p95 rotation residual `5.59 deg`. Treat this as evidence that
  the bag can stress the future dynamic update, not as ground truth arm pose.

The plan should cover:
- Which OpenVINS node/component should consume the dynamic update.
- Whether to consume `DynamicMarkerObservation` directly, a head-derived arm
  pose message, or a new explicit dynamic-arm-pose message.
- Exact transforms between head pose, head camera, marker ID2, arm camera, and
  arm IMU.
- Covariance propagation from head OpenVINS pose, dynamic marker observation,
  and `arm_marker_extrinsics.yaml`.
- Timing synchronization and buffering between head pose, dynamic observation,
  and the arm estimator state time.
- Innovation gating, outlier rejection, stability gates, and recovery behavior.
- Whether the update should constrain arm camera pose or arm IMU pose, and how
  it maps into the OpenVINS state.
- How to avoid double-counting correlated information from fixed marker ID0.
- Launch parameters, topics, and default-off/default-on behavior.
- Replay and live validation steps, including expected topics/messages/TF.
- Risks, rollback, and acceptance criteria.
```
