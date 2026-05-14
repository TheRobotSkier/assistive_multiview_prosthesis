# Future Tasks

## Current Phase 2 Status

As of 2026-05-13, the single dynamic ID2 live workflow has been changed to
default to `active` in
`docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml`.
That means the one-command launch enables fixed ID0 updates/reanchor plus
dynamic ID2 normal updates and guarded dynamic ID2 initial-lock/reanchor by
default. The fixed marker map remains ID0-only; do not add ID2 to
`marker_fixed_ids`.

D435i color pointclouds are opt-in on the dynamic ID2 live launch with
`enable_pointclouds:=true`. OpenVINS RGB remains 30 Hz, pointcloud depth is
15 Hz, and the grasping topics are `/head/d435i_head/points_marker_map` and
`/arm/d435i_arm/points_marker_map` in `marker_map` once OpenVINS camera TF is
available.
As of 2026-05-14, the live D435i OpenVINS launches default
`hold_back_imu_for_frames:=true` to keep RealSense image/IMU publication order
chronological under Jetson load. X86-side segmentation pointcloud work is
tracked in `.agents/x86_segmentation_pointcloud_plan.md`.

Final 2026-05-14 pointcloud status: RViz2 color pointclouds work and OpenVINS
still runs well. The pointcloud status topic reporting alternating `published`
and `rate_limited` is expected with the output rate cap.

Important current caveat: the latest live ID2 measurement bag
`dynamic_id2_arm_update_live_20260513_125316` validated the measurement path
but not the OpenVINS update/reanchor path. It recorded `218` head dynamic ID2
observations and `176` finite dynamic arm-pose observations with good quality,
but `/ov_msckf_arm/odomimu` stopped before the dynamic arm-pose measurements
started, so `/ov_msckf_arm/dynamic_arm_update/status` had `0` messages. Before
judging final active behavior, record a bag where arm odom, dynamic arm-pose
observations, and OpenVINS dynamic update status overlap in time.

Current dynamic ID2 user docs:
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_parameters.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_validation_commands.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/dynamic_id2_arm_update_live_bag_recommendations.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_marker_id2_extrinsic_calibration_commands.md`

Marker ID2 fixture note, 2026-05-13: the next physical ID2/arm-D435i fixture
needs a redesign because the prosthesis can occlude the marker from the head
camera. After the redesigned mount is ready, use the focused calibration command
sheet above to record an observation-mode bag, update
`config/markers/arm_marker_extrinsics.yaml`, then record an active validation
bag with overlapping `/ov_msckf_arm/odomimu`,
`/arm/marker_pose/dynamic_arm_pose_observation`, and
`/ov_msckf_arm/dynamic_arm_update/status`.

As of 2026-05-11, the Phase 2 head marker EKF path and arm D435i Phase 2
bringup have been committed and pushed. Latest local commit seen by Codex:

```text
374216c Add arm D435i Phase 2 OpenVINS bringup
```

The 100 mm marker covariance bag analysis has been completed. Phase 2
OpenVINS-internal marker update/reset code has been implemented locally and
passed the low-memory Docker/Jazzy build, launch/message smoke checks, live
head-camera testing, arm setup smoke checks, and pre-init raw replay validation
for both head and arm D435i cameras.

Validated Phase 2 head D435i replay bag:

```text
docker_ws/bags/openvins_tests/phase2_live/head_marker_phase2_100mm_preinit_20260510_122723
```

Fresh replay of only raw image/camera-info/IMU topics initialized OpenVINS,
published `poseimu`, `odomimu`, and `pathimu` in `marker_map`, produced `4`
marker-map resets and `1212` accepted marker-0 EKF updates, and did not reproduce
the live Propagator assertion.

Validated Phase 2 arm D435i replay bag:

```text
docker_ws/bags/openvins_tests/phase2_live/arm_marker_phase2_100mm_preinit_20260510_142201
```

Fresh replay of only raw arm image/camera-info/IMU topics initialized OpenVINS,
published `/ov_msckf_arm/poseimu`, `/ov_msckf_arm/odomimu`, and
`/ov_msckf_arm/pathimu` in `marker_map`, produced `3` marker-map resets and
`1664` accepted marker-0 EKF updates, and did not reproduce the live hard
Propagator assertion.

Use the current Phase 2 handoff note before continuing:
`.agents/phase2_openvins_handoff_status.md`.

Repository hygiene:
- generated `build_overlay/`, `install_overlay/`, and `log_overlay/` artifacts
  should stay out of commits
- ROS bags should stay local validation artifacts unless a separate data-sharing
  decision is made

Near-term next work:
- Per-instance OpenVINS TF frame support has been live-smoke checked: head and
  arm publish separate `/ov_msckf` and `/ov_msckf_arm` topics, `poseimu` headers
  remain in `marker_map`, odom child frames are `head_imu` and `arm_imu`, marker
  observations target `head_imu` and `arm_imu`, and TF exposes the intended
  `marker_map -> *_imu -> *_cam0` frames.
- The Phase 2 marker-node 15 Hz throttle improved RViz responsiveness and VIO
  feature quality, but did not fully prevent the OpenVINS `Propagator.cpp`
  assertion when RViz and VS Code were open. MAXN SUPER power mode let the
  system run longer before the failure, which supports a load/timing sensitivity
  hypothesis but does not prove marker detection is the only bottleneck.
- A no-RViz dual-camera validation bag has now been recorded at
  `docker_ws/bags/openvins_tests/phase2_live/dual_openvins_tf_phase2_20260511_160757`.
  A read-only `ros2 bag info` check confirmed 68.0 s of both head/arm raw inputs,
  marker observations, OpenVINS outputs, `/tf`, `/tf_static`, and `/rosout`.
  Replay sampling confirmed head/arm odom child frames, pose headers, marker
  target frames, marker ID 0 observations, and non-colliding head/arm TF child
  frames.
  This bag did not include the head camera seeing arm-mounted marker ID 2, so it
  validates the dual stack/recording workflow but is not sufficient for the ID 2
  extrinsic calibration.
- A no-RViz arm-mounted ID2 calibration bag has now been recorded at
  `docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816`.
  Metadata and replay sampling show the expected dual-stack topics and frames.
  Offline ArUco detection on raw images confirmed head marker ID 0, head
  arm-mounted marker ID 2, and arm marker ID 0 visibility. Use this as the
  primary source bag for the first dynamic-marker calibration implementation.
  The earlier RViz attempt,
  `dual_openvins_id2_calib_phase2_20260511_164627`, is shorter and should be
  treated as secondary/interrupted.
- The dynamic marker ID2 observation and offline arm-marker calibration path is
  now implemented locally but not yet committed. New validation recipe:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md`.
  The implementation adds `DynamicMarkerObservation`, publishes head-visible ID2
  only on `/head/marker_pose/dynamic_observation`, keeps fixed observations on
  `/head/marker_pose/observation` and `/arm/marker_pose/observation` as ID0
  only, and saves the first calibration to
  `config/markers/arm_marker_extrinsics.yaml`. The known bag produced `230`
  synchronized samples, `173` inliers, `57` outliers, p95 translation residual
  `0.0273 m`, and p95 rotation residual `3.541 deg`. Online arm pose/state
  updates are still intentionally not implemented.
- A live dynamic-ID2 validation bag has now been recorded at
  `docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_20260511_194110`.
  It confirms the runtime topic split: head dynamic observations are marker ID2
  only, head/arm fixed observations are marker ID0 only, and OpenVINS head/arm
  frames remain separated. It is not strong enough to replace the checked-in
  calibration because only `72` synchronized calibration samples and `68`
  inliers were available, below the first accepted-calibration gate of `100`
  inliers. A lower-threshold characterization gave p95 translation residual
  `0.0252 m` and p95 rotation residual `2.067 deg`.
- A debug-only head-derived arm D435i pose preview has now been implemented
  locally but not yet committed. It consumes
  `/head/marker_pose/dynamic_observation`, `/ov_msckf/poseimu`, and
  `config/markers/arm_marker_extrinsics.yaml`, then publishes only
  `/arm/marker_pose/head_derived/*` topics and `_head_preview` TF frames. It
  does not publish `/arm/marker_pose/observation`, does not change
  `marker_fixed_ids`, and does not feed OpenVINS. Validation recipe:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/head_derived_arm_pose_preview_validation.md`.
- The first live preview validation bag has been recorded at
  `docker_ws/bags/openvins_tests/phase2_live/head_derived_arm_preview_20260512_073918`.
  Read-only analysis showed `67.12 s`, `56.0 MiB`, `37943` messages,
  `162` dynamic ID2 observations, `48` accepted preview poses in `marker_map`,
  finite preview covariance, and `_head_preview` TF frames. Dynamic marker
  quality was good: median reprojection `0.252 px`, p95 reprojection
  `0.535 px`, median distance `0.582 m`, and p95 view angle `16.95 deg`.
  Compared with arm OpenVINS converted from `arm_imu` to the arm camera frame,
  `42` matched samples had median translation residual `0.0646 m`, p95
  translation residual `0.1978 m`, median rotation residual `2.06 deg`, and p95
  rotation residual `3.69 deg`. This validates the debug preview path but is
  not a complete raw replay source because it did not record raw camera/IMU
  streams or fixed marker observation topics.
- A complete raw dynamic-ID2 update planning bag has now been recorded at
  `docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_full_raw_20260512_090704`.
  Read-only validation showed `62.39 s`, `2.9 GiB`, `71417` messages, both
  head/arm raw color image, camera-info, and IMU streams, fixed observations
  with ID0 only (`525` head targeting `head_imu`, `302` arm targeting
  `arm_imu`), `111` dynamic ID2 observations, `18` accepted preview poses in
  `marker_map`, separated OpenVINS outputs with odom children `head_imu` and
  `arm_imu`, and expected head/arm plus `_head_preview` TF frames. Raw-image
  calibration characterization was strong: head ID0 `1578`, head ID2 `373`,
  head ID0+ID2 `373`, arm ID0 `822`, `159` synchronized samples, `154` inliers,
  p95 translation residual `0.0120 m`, and p95 rotation residual `3.839 deg`.
  Preview-vs-current-arm-OpenVINS camera comparison had only `17` matched
  accepted preview samples and large translation disagreement
  (median `1.023 m`, p95 `22.11 m`), so do not treat the arm OpenVINS path in
  this bag as ground truth. This is now the preferred local source bag for
  planning/replay-testing a future dynamic-ID2 arm update.

## A. Arm D435i Setup From Calibration Output

Calibration files are available at:
`docker_ws/calibration/arm/d435i_310622071850/`

Arm Phase 2 setup has been implemented by mirroring the head D435i
configuration where appropriate.

Planned values:
- Namespace: `arm`
- Camera name: `d435i_arm`
- RealSense serial: `_310622071850`
- Mount location: `prosthetic_arm`

Implemented deliverables:
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imucam_chain.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/kalibr_imu_chain.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`
- `multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py`
- `multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_marker_validation.md`
- `multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_live_trial_and_recording.md`

The arm OpenVINS node uses `/ov_msckf_arm` to avoid topic collisions with head
OpenVINS. The Phase 2 launch now uses per-instance OpenVINS frames so arm TF can
run beside head TF: `marker_map -> head_imu -> head_cam0` and
`marker_map -> arm_imu -> arm_cam0`.

## B. OpenVINS Internal Marker Update / EKF Reanchor

Phase 2 head D435i internal marker update/reanchor is implemented and validated.
Keep future edits here focused on behavior improvements found during head or arm
validation.

Phase 2 target behavior:
- accepted observations of known fixed markers should become OpenVINS EKF
  updates, not only an external corrected-odom wrapper
- marker ID 0 should define the shared `marker_map` reference frame at startup
  or reanchor, so later head and arm D435i instances can publish point clouds in
  a common frame
- when VIO is healthy, marker updates should improve global pose consistency and
  keep state uncertainty appropriately bounded
- after VIO drift, marker reacquisition should repair pose while handling
  covariance and velocity consistently enough to avoid the repeated Phase 1
  snap-back behavior

Use `.agents/marker_pose_covariance_plan.md` as the starting point for marker
measurement covariance, innovation gating, EKF update design, and velocity
handling. Do not assume a pose-only update is sufficient if the internal VIO
velocity has already become inconsistent.

Known robustness follow-up:
- Repeated dual live runs hit the OpenVINS `Propagator.cpp` assertion comparing
  requested propagation duration against summed IMU dt. A 2026-05-14 retry keeps
  RealSense/OpenVINS at 30 fps, caps only the internal stale camera queue, and
  turns incomplete IMU coverage into `[PROP]` diagnostics instead of a hard
  assertion. After repeated `[PROP]` messages still appeared from a stale
  timestamp, both D435i configs were switched to the Jetson-safe profile
  (`track_frequency: 21.0`, `num_pts: 200`, `fast_threshold: 25`,
  `min_px_dist: 15`, `max_clones: 8`, `max_slam: 25`,
  `max_msckf_in_update: 25`, `num_opencv_threads: 2`).
- After marker throttling, failures still occurred under heavy desktop load
  with RViz and VS Code open; the same setup ran longer in MAXN SUPER mode. The
  next OpenVINS robustness fix should avoid accepting intermittent VIO process
  death. Better candidates are a deliberate Jetson-safe tuning profile,
  camera-frame backlog control before propagation, newest-frame processing when
  the estimator is behind, and a recovery path that skips stale camera updates
  without freezing the state time.
- A stable no-RViz dual-camera bag has been recorded, so planning the
  arm-mounted marker ID 2 observation/calibration path is reasonable. Still avoid
  implementing online arm state updates until an ID2-visible calibration bag has
  been collected and the dynamic-marker path is separated from fixed-marker EKF
  updates.
- The ID2-visible calibration bag is now available, so the next implementation
  can start with an offline calibration pipeline and raw dynamic-marker
  observation path. Online arm-state updates should still wait until the
  calibration script reports a stable `T_armcam_marker2` estimate with residuals
  and uncertainty.

## C. VIO Health Monitor And Reset/Reinitialize Behavior

Improve logic that marks VIO invalid when covariance, velocity, workspace
bounds, or marker innovation become unreasonable. This can remain external
first, then potentially move into OpenVINS after the behavior is trusted.

## D. Two-Camera Point-Cloud Fusion

Use `marker_map` or another common reference frame to align head and arm D435i
point clouds. Avoid fusing clouds directly in drifting raw OpenVINS `global`
frames.

Immediate prerequisite:
- The per-instance OpenVINS TF frame support has been live-smoke checked, a
  no-RViz dual-camera validation bag has been recorded, an ID2-visible
  calibration bag is available, and the offline ID2-to-arm-D435i calibration
  path is implemented locally. Before online two-camera fusion, run the live
  dynamic-ID2 validation recipe, record a fresh bag including
  `/head/marker_pose/dynamic_observation`, and confirm the new bag calibrates
  with stable residuals.

Planned final marker layout:
- Fixed world/common reference marker: 6x6 marker ID 0, 100 mm x 100 mm, in
  front of the user and visible to the head D435i.
- Arm-mounted marker: 6x6 marker ID 1 or ID 2, 100 mm x 100 mm, rigidly
  attached to the prosthetic arm with a fixed transform to the arm D435i. The
  current live-test candidate is ID 2.
- Optional extra marker: another 6x6 marker, 100 mm x 100 mm, for calibration or
  redundancy experiments.

Important modeling boundary:
- Do not add the arm-mounted marker ID 1/ID 2 as a fixed `marker_map` landmark
  in the current fixed-marker update path. The current marker map assumes listed
  markers are stationary in the map. Arm-mounted markers move with the arm and
  need a separate dynamic-marker/multiview fusion path that can publish raw
  dynamic-marker observations without applying them as fixed world updates.

Future multiview fusion concept:
- Head D435i updates its pose from VIO and fixed marker ID 0.
- Arm D435i updates its pose from VIO and fixed marker ID 0 when visible.
- When the head D435i sees the arm-mounted marker, use the head pose
  uncertainty, marker detection uncertainty, and calibrated
  `T_arm_camera_marker` to update or constrain the arm D435i pose. This should
  account for the arm OpenVINS uncertainty instead of treating the head-derived
  pose as ground truth.

Next planning target:
- Plan the dynamic ID2 arm OpenVINS update path before coding. The update should
  consume a head-derived arm pose measurement as an arm-camera or arm-IMU pose
  constraint, transform it into the arm OpenVINS state frame, propagate head,
  dynamic-marker, and extrinsic covariance, apply timing and innovation gates,
  and remain separate from the fixed marker-map landmark path. The preview bag
  above is useful characterization evidence; use
  `dynamic_id2_arm_update_full_raw_20260512_090704` as the main local full raw
  replay-style validation bag, with `dual_openvins_id2_dynamic_phase2_20260511_194110`
  as an older runtime comparison.

## E. Arm Marker To Arm D435i Extrinsic Calibration

The calibration script for the fixed transform between the arm-mounted marker
and the arm D435i is implemented locally as:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py
```

The current candidate marker is ID 2, and the implementation remains
configurable so ID 1 or another marker can be used later.

One possible calibration procedure:
- Place fixed marker ID 0 where both cameras can observe it.
- Mount marker ID 2 rigidly on the arm D435i/arm assembly and keep it visible to
  the head D435i during calibration samples.
- Move the prosthetic arm through multiple poses.
- Save frames when the head D435i sees marker ID 0 and the arm-mounted marker,
  and the arm D435i sees marker ID 0.
- Estimate the rigid transform between the arm-mounted marker and the arm D435i
  from repeated simultaneous observations.
- Report residuals and uncertainty so the transform can be used in later
  multiview fusion.

Useful calibration relationship:
- If fixed marker ID 0 gives `T_map_headcam` and `T_map_armcam`, and the head
  camera detects the arm-mounted marker as `T_headcam_marker`, then compute each
  sample as `T_armcam_marker = inv(T_map_armcam) * T_map_headcam *
  T_headcam_marker`.
- Collect time-synchronized samples, reject outliers, compute a robust
  mean/median SE(3) transform, and save residual statistics/covariance with the
  extrinsic config.

Primary calibration-source bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816
```

Raw-image ArUco check:
- head ID 0: 468 frames
- head ID 2: 1035 frames
- arm ID 0: 1457 frames

First offline calibration result from this bag:
- synchronized samples: 230
- inliers/outliers: 173 / 57
- median translation residual: 0.0130 m
- p95 translation residual: 0.0273 m
- median rotation residual: 1.801 deg
- p95 rotation residual: 3.541 deg
- saved config: `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml`
- validation recipe:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md`

Latest live runtime validation bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_20260511_194110
```

This 92.26 s bag validates that ID2 is published only on the dynamic topic and
that fixed marker observations remain ID0-only. It should not replace the
checked-in extrinsic config because arm ID0 visibility overlapped the head
ID0+ID2 frames too rarely: `72` synchronized samples, `68` inliers with the
lower `--min-inliers 50` characterization, and failure at the normal
`--min-inliers 100` gate.

Known calibration warnings:
- Head Kalibr intrinsics differ from bag CameraInfo by about `5.094 px`.
- Arm Kalibr intrinsics differ from bag CameraInfo by about `3.863 px`.
- Treat these as expected warnings for the current known bag unless running the
  script with `--strict-camera-info`.

## F. Arm D435i Trajectory Prediction With Uncertainty

Investigate a prediction node that runs on each arm D435i image or synchronized
state update and predicts the arm camera feature/pose trajectory over a short
horizon, for example one second.

The output should include:
- predicted poses along the horizon
- uncertainty for each predicted pose
- enough timing metadata to fuse predictions with marker/VIO updates

This is after Phase 1 covariance validation and likely after the first
two-camera fusion prototype.
