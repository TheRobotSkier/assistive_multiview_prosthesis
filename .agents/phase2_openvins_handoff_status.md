# Phase 2 OpenVINS Marker EKF Handoff Status

Use this note when starting a new Codex chat after the initial Phase 2
implementation work.

## Current Status

As of 2026-05-13, the dynamic ID2 arm update workflow exists as the default
one-command live path:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/dynamic_id2_arm_update.yaml
```

`dynamic_id2_arm_update.yaml` now defaults `launch.mode: active`. Effective
default behavior is fixed ID0 updates/reanchor plus dynamic ID2 normal arm
updates and guarded dynamic ID2 initial-lock/reanchor. The low-level fixed
marker maps remain ID0-only; ID2 must stay out of `marker_fixed_ids`.

Color pointclouds are now opt-in on the same live path with
`enable_pointclouds:=true`. OpenVINS RGB input remains `640x480x30`; depth and
color pointcloud generation use `640x480x15` depth and republish transformed
grasping clouds on `/head/d435i_head/points_marker_map` and
`/arm/d435i_arm/points_marker_map` in `marker_map` after OpenVINS has initialized
the camera TF. The default does not require the explicit lock topic; it publishes
whenever marker-map TF is available.
Validation commands live in
`docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md`.

Final 2026-05-14 validation: color pointclouds are visible in RViz2 and OpenVINS
still runs well. `/head/d435i_head/points_marker_map/status` reported
`accepted:true` with `reason:"published"` interleaved with expected
`reason:"rate_limited"` messages.

As of 2026-05-14, the live D435i OpenVINS launch path defaults
`hold_back_imu_for_frames:=true` for both RealSense nodes. This is intended to
keep image and IMU publication order chronological under Jetson load. Compare
with `hold_back_imu_for_frames:=false` only when diagnosing the OpenVINS
`Propagator.cpp` timing assert.

Later on 2026-05-14, an OpenVINS image subscriber sensor-data QoS experiment was
rolled back after both marker-enabled OpenVINS nodes exited at startup with
`double free or corruption`. The retained OpenVINS stability changes are:
const-reference `get_params()` access, a small internal stale-camera-frame cap,
and non-fatal `[PROP]` diagnostics instead of the old `Propagator.cpp`
assertion. Rebuild `ov_msckf` sequentially before testing these changes.

After the first improved baseline still showed repeated `[PROP]` messages from a
stale timestamp, both D435i OpenVINS configs were switched to a Jetson-safe
dual-camera profile: `track_frequency: 21.0`, `num_pts: 200`,
`fast_threshold: 25`, `min_px_dist: 15`, `max_clones: 8`, `max_slam: 25`,
`max_msckf_in_update: 25`, and `num_opencv_threads: 2`. RealSense RGB remains
`640x480x30`, so no recalibration is implied by this tuning step.

Pointcloud root-cause fix: custom D435i launches were explicitly updated to set
`enable_infra`, `enable_infra1`, and `enable_infra2` false. Without this, the
RealSense driver opened extra infrared streams and depth/pointcloud frames timed
out. The delayed `enable_pointcloud_neon_fix` default is now false; startup
`pointcloud__neon_` parameters are sufficient.

Rollback/diagnostic controls:

```text
mode:=observe         dynamic measurement publication/status only
mode:=update          normal dynamic EKF updates, dynamic reanchor disabled
mode:=would_reanchor  would-initial-lock/reanchor status without mutation
use_dynamic_arm_pose_updates:=false  full dynamic-disable fallback
```

Recent live validation status:
- `dynamic_id2_arm_update_live_20260513_095416`: active launch ran, but no ID2
  dynamic observations were published.
- `dynamic_id2_arm_update_live_20260513_125316`: measurement path worked with
  `218` head dynamic ID2 observations and `176` finite dynamic arm-pose
  observations; median reprojection `0.230 px`, p95 reprojection `0.555 px`,
  median distance `0.597 m`, p95 view angle `47.4 deg`. It did not validate
  OpenVINS dynamic update/reanchor because `/ov_msckf_arm/odomimu` stopped
  before ID2 measurements began, leaving `/ov_msckf_arm/dynamic_arm_update/status`
  at `0` messages.

Before changing or judging final behavior, verify that `/ov_msckf_arm/odomimu`,
`/arm/marker_pose/dynamic_arm_pose_observation`, and
`/ov_msckf_arm/dynamic_arm_update/status` overlap in a live bag.

Focused ID2 arm-marker extrinsic calibration commands live here:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_marker_id2_extrinsic_calibration_commands.md
```

Physical fixture note, 2026-05-13: the next arm-mounted ID2 fixture needs a
redesign because the prosthesis can occlude marker ID2 from the head camera.
Once the updated design is ready, refresh
`config/markers/arm_marker_extrinsics.yaml` using the focused calibration
command sheet before judging final active dynamic ID2 update/reanchor behavior.

As of 2026-05-11, the Phase 2 head marker EKF path and the arm D435i Phase 2
bringup have been committed and pushed. Latest local commit seen by Codex:

```text
374216c Add arm D435i Phase 2 OpenVINS bringup
```

Phase 2 marker-enabled OpenVINS implementation has been added locally and the
low-memory Jazzy/Docker build completed successfully on the Jetson.

Successful build checkpoint:

```text
Summary: 6 packages finished [43min 20s]
  3 packages had stderr output: ov_core ov_init ov_msckf
```

The stderr note was OpenVINS/CMake/Eigen warnings, not a colcon failure.

Successful smoke validation:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  "source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 interface show sensor_fusion_msgs/msg/MarkerPoseObservation && \
   test -x install_overlay/ov_msckf/lib/ov_msckf/run_subscribe_msckf_marker && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py --show-args"
```

This command exited successfully after showing the marker message, the
`run_subscribe_msckf_marker` executable, and both launch argument listings.

2026-05-08 replay validation used only confirmed 100 mm marker data with
`head_aruco_map.yaml` (`marker 0: size_m: 0.100`). Older
`head_marker_phase1*_20260506_*` bags and the `1533mm`/`160mm` configs were
excluded.

Four requested 100 mm bags were replayed in order using only raw image,
camera-info, and IMU topics. Marker observations were produced for all four
bags. OpenVINS initialized only on
`head_marker_cov_validation_100mm_final_lost_regained_vio_good_20260507_140742`;
that replay published `poseimu`, `odomimu`, and `pathimu` in `marker_map`,
performed one marker-0 first-lock reset, then accepted bounded marker-0 EKF
updates. The stationary, gentle-motion, and VIO-drift raw replays did not
initialize OpenVINS, so they did not exercise the internal marker EKF path.

Marker ID 1 was validated with image-synchronized synthetic observations on the
VIO-good replay: OpenVINS logged seven non-fixed-marker rejections and zero
marker-1 accepts or resets. No source patch to `ROS2Visualizer` marker queue
handling was indicated by these runs.

2026-05-10 live testing looked good: after larger VIO drift/reacquire events,
marker ID `0` sometimes took a little time to pull the estimate back, but it did
recover the correct pose and continue marker EKF updates. A new pre-init replay
validation bag was recorded:

```text
docker_ws/bags/openvins_tests/phase2_live/head_marker_phase2_100mm_preinit_20260510_122723
```

Fresh replay of only raw image/camera-info/IMU topics initialized OpenVINS,
published `poseimu`, `odomimu`, and `pathimu` in `marker_map`, produced `4`
marker-map resets and `1212` accepted marker-0 EKF updates, and did not reproduce
the live Propagator assertion.

The arm D435i Phase 2 setup was then added from
`docker_ws/calibration/arm/d435i_310622071850/` with OpenVINS output under
`/ov_msckf_arm` and marker output under `/arm/marker_pose`.

Per-instance OpenVINS ROS 2 frame parameters have now been implemented for Phase
2 so head and arm OpenVINS can publish TF simultaneously:

```text
marker_map -> head_imu -> head_cam0
marker_map -> arm_imu  -> arm_cam0
```

2026-05-11 no-RViz dual-camera validation recording:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_tf_phase2_20260511_160757
```

The bag was recorded from the full dual-camera Phase 2 stack without RViz. A
read-only `ros2 bag info` check showed a 68.0 s, 3.4 GiB MCAP bag with both
head/arm raw image, camera-info, and IMU streams; both head/arm marker
observation streams; both `/ov_msckf` and `/ov_msckf_arm` pose/odom/path output
groups; `/tf`; `/tf_static`; and `/rosout`. This is the current best
no-RViz/no-GUI-load checkpoint for the dual-camera TF and marker-throttle
workflow. It was not an arm-mounted marker calibration bag: the head camera did
not see marker ID 2 during the recording.

Replay sampling of recorded outputs confirmed:
- `/ov_msckf/odomimu.child_frame_id == head_imu`
- `/ov_msckf_arm/odomimu.child_frame_id == arm_imu`
- `/ov_msckf/poseimu.header.frame_id == marker_map`
- `/ov_msckf_arm/poseimu.header.frame_id == marker_map`
- `/head/marker_pose/observation.target_frame == head_imu`
- `/arm/marker_pose/observation.target_frame == arm_imu`
- sampled marker observations were marker ID 0 for both head and arm
- sampled `/tf` contained `head_imu`, `head_cam0`, `arm_imu`, and `arm_cam0`
  and no generic OpenVINS `imu` or `cam0` child frames

2026-05-11 arm-mounted marker ID2 calibration recording:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816
```

This is the current best calibration-source bag. It was recorded without RViz
after an earlier RViz attempt triggered the live OpenVINS timing failure. A
read-only `ros2 bag info` check showed a 53.5 s, 2.7 GiB MCAP bag with both
head/arm raw image, camera-info, and IMU streams; both head/arm marker streams;
both `/ov_msckf` and `/ov_msckf_arm` output groups; `/tf`; `/tf_static`; and
`/rosout`.

Replay sampling confirmed the same validated TF/topic frame behavior:
`head_imu` and `arm_imu` odom child frames, `marker_map` pose headers,
`head_imu` and `arm_imu` marker targets, and no generic OpenVINS `imu` or
`cam0` TF child frames in the sampled replay stream.

Offline ArUco detection on the recorded raw images using the same
`DICT_6X6_1000` dictionary confirmed:
- head raw images: marker ID 0 detected in 468 frames
- head raw images: arm-mounted marker ID 2 detected in 1035 frames
- arm raw images: marker ID 0 detected in 1457 frames

The recorded `/head/marker_pose/observation` and `/arm/marker_pose/observation`
streams still report marker ID 0, as expected. Marker ID 2 is present in the raw
head images and should be consumed by the future dynamic-marker
observation/calibration path, not by the fixed `marker_map` EKF update path.

The dynamic marker ID2 observation and offline calibration path has now been
implemented locally but is not yet committed. It adds
`sensor_fusion_msgs/msg/DynamicMarkerObservation.msg`, publishes head-visible
ID2 as `/head/marker_pose/dynamic_observation`, keeps ID2 off the fixed
`/head/marker_pose/observation` topic, and adds
`sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py`. The first
calibration from this bag is saved at:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
```

Known offline result:
- synchronized samples: `230`
- inliers/outliers: `173 / 57`
- p95 translation residual: `0.0273 m`
- p95 rotation residual: `3.541 deg`
- expected CameraInfo/Kalibr warnings: head `5.094 px`, arm `3.863 px`

Latest live dynamic-ID2 validation bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_20260511_194110
```

Read-only validation showed `92.26 s`, `4.3 GiB`, and `110360` messages. The
dynamic path published `320` `/head/marker_pose/dynamic_observation` messages,
all marker ID `2` in `head_d435i_head_color_optical_frame`. The fixed marker
paths stayed ID0-only: `763` head fixed observations targeting `head_imu` and
`367` arm fixed observations targeting `arm_imu`. OpenVINS odom child frames
remained `head_imu` and `arm_imu`, and TF kept the expected head/arm frame
split.

This live bag is good runtime evidence but not a replacement calibration source.
It had `72` synchronized calibration samples and `68` inliers, below the normal
`--min-inliers 100` acceptance gate. A lower-threshold characterization produced
p95 translation residual `0.0252 m` and p95 rotation residual `2.067 deg`.

The debug-only head-derived arm D435i pose preview is now implemented locally
but not yet committed. It consumes `/head/marker_pose/dynamic_observation`,
the head OpenVINS pose `/ov_msckf/poseimu`, and
`config/markers/arm_marker_extrinsics.yaml`, then publishes only:

```text
/arm/marker_pose/head_derived/arm_camera_pose
/arm/marker_pose/head_derived/arm_camera_body_pose
/arm/marker_pose/head_derived/arm_marker_pose
/arm/marker_pose/head_derived/path
/arm/marker_pose/head_derived/status
```

It also publishes `_head_preview` TF frames for RViz. It does not publish
`/arm/marker_pose/observation`, does not modify `marker_fixed_ids`, and does
not feed OpenVINS.

Live preview validation bag:

```text
docker_ws/bags/openvins_tests/phase2_live/head_derived_arm_preview_20260512_073918
```

Read-only validation showed `67.12 s`, `56.0 MiB`, and `37943` messages. The
bag contains `162` dynamic ID2 observations, `48` accepted preview camera poses
in `marker_map`, finite preview covariance, `991` head pose samples, `1184` arm
pose samples, and the expected `_head_preview` TF frames. Dynamic marker quality
was good: median reprojection `0.252 px`, p95 reprojection `0.535 px`, median
distance `0.582 m`, and p95 view angle `16.95 deg`.

Comparison against arm OpenVINS converted from `arm_imu` to the arm camera frame
found `42` matched preview samples with median translation residual `0.0646 m`,
p95 translation residual `0.1978 m`, median rotation residual `2.06 deg`, and
p95 rotation residual `3.69 deg`. This validates the debug preview path and
supports planning a gated dynamic-ID2 arm update. It is not a complete raw
replay source because it did not record raw camera/IMU streams or fixed marker
observation topics.

Complete raw dynamic-ID2 update planning bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_full_raw_20260512_090704
```

Read-only validation showed `62.39 s`, `2.9 GiB`, and `71417` messages. It
contains both head/arm raw color image, camera-info, and IMU streams; fixed
marker observations with ID0 only (`525` head targeting `head_imu`, `302` arm
targeting `arm_imu`); `111` dynamic ID2 observations; `18` accepted preview
poses in `marker_map`; head/arm OpenVINS outputs with odom child frames
`head_imu` and `arm_imu`; `/tf`; and `/tf_static`. Raw image and IMU stamps
were monotonic in the validation scan.

Dynamic marker quality in this bag was good: stable observations `104 / 111`,
median reprojection `0.274 px`, p95 reprojection `0.604 px`, median distance
`0.528 m`, and p95 view angle `22.00 deg`. Offline raw-image calibration
characterization found head ID0 `1578`, head ID2 `373`, head ID0+ID2 `373`, arm
ID0 `822`, `159` synchronized samples, `154` inliers, p95 translation residual
`0.0120 m`, and p95 rotation residual `3.839 deg`.

Preview-vs-current-arm-OpenVINS camera comparison on this bag had only `17`
matched accepted preview samples and large translation disagreement: median
`1.023 m`, p95 `22.11 m`; rotation residual median `2.97 deg`, p95 `5.59 deg`.
This likely reflects the current arm state disagreement/drift that the dynamic
ID2 update is intended to constrain. Do not use the current arm OpenVINS path in
this bag as ground truth. This is the preferred local full raw replay bag for
planning and validating the future dynamic-ID2 arm update.

Validation recipe for live tests and bag recording:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md
```

Preview validation recipe:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/head_derived_arm_pose_preview_validation.md
```

Earlier interrupted/secondary ID2 bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164627
```

This 22.8 s bag was recorded during the RViz attempt and should not be the
primary calibration source.

Validated Phase 2 arm D435i replay bag:

```text
docker_ws/bags/openvins_tests/phase2_live/arm_marker_phase2_100mm_preinit_20260510_142201
```

Fresh replay of only raw arm image/camera-info/IMU topics initialized OpenVINS,
published `/ov_msckf_arm/poseimu`, `/ov_msckf_arm/odomimu`, and
`/ov_msckf_arm/pathimu` in `marker_map`, produced one first marker-map lock,
`3` total marker-map resets, and `1664` accepted marker-0 EKF updates. The raw
timestamp scan was monotonic for arm image, camera-info, IMU, marker, and
OpenVINS output topics. Replay logged one missing-inertial-measurement
Propagator warning, but no fatal error and no hard Propagator assertion.

## Implemented Pieces

- New ROS 2 message package:
  `docker_ws/multi_cam_localization/sensor_fusion_msgs`
- Marker observation message:
  `sensor_fusion_msgs/msg/MarkerPoseObservation.msg`
- Dynamic marker observation message:
  `sensor_fusion_msgs/msg/DynamicMarkerObservation.msg`
- Python Phase 1 marker node now publishes:
  `/head/marker_pose/observation`
- Python marker node now also publishes dynamic head-visible arm marker
  observations on:
  `/head/marker_pose/dynamic_observation`
- Phase 2-only marker OpenVINS executable:
  `run_subscribe_msckf_marker`
- Phase 2 launch files:
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_marker_pose_phase2.launch.py`
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py`
- Phase 2 validation recipe:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_marker_validation.md`
- Arm Phase 2 validation recipes:
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_marker_validation.md`
  - `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_live_trial_and_recording.md`
- Dynamic ID2 validation recipe:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md`
- Head-derived arm pose preview validation recipe:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/head_derived_arm_pose_preview_validation.md`
- Offline arm marker calibration script:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py`
- Debug-only head-derived arm pose preview node:
  `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/head_derived_arm_pose_preview_node.py`
- OpenVINS marker update/reset code was added under:
  `docker_ws/src/open_vins/ov_msckf`

## Important Warnings

Do not run heavy OpenVINS builds casually on the Jetson. If an OpenVINS build
is needed, close other applications and use sequential low-memory settings:

```bash
MAKEFLAGS=-j1 CMAKE_BUILD_PARALLEL_LEVEL=1
colcon --log-base log_overlay build --executor sequential --parallel-workers 1 ...
```

Generated overlay directories must not be committed:

```text
docker_ws/build_overlay/
docker_ws/install_overlay/
docker_ws/log_overlay/
```

`docker_ws/src/open_vins` has been converted locally from the broken parent-repo
gitlink into a lean vendored source tree. The staged vendor copy includes
`ov_core`, `ov_init`, `ov_msckf`, `config`, and top-level license/readme
metadata. Large optional OpenVINS data/evaluation/docs assets are ignored.

## Files To Read In The Next Chat

- `.agents/AGENTS.md`
- `.agents/phase2_openvins_handoff_status.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_marker_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/head_derived_arm_pose_preview_validation.md`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_ekf_handoff_prompt.md`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/head_derived_arm_pose_preview_node.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_marker_pose_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_derived_arm_pose_preview.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.h`
- `docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.h`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManager.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/core/VioManagerOptions.h`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h`
- `docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp`
- `docker_ws/src/open_vins/ov_msckf/src/run_subscribe_msckf_marker.cpp`
- `docker_ws/src/open_vins/ov_msckf/cmake/ROS2.cmake`
- `docker_ws/src/open_vins/ov_msckf/package.xml`

## Next Task

Plan the next feature on top of the validated dynamic-ID2 topic split: a
head-derived arm D435i pose preview path that consumes
`/head/marker_pose/dynamic_observation` plus
`config/markers/arm_marker_extrinsics.yaml`, publishes a visualization/debug
pose for the arm camera in `marker_map`, and does not feed OpenVINS or perform
online arm state updates yet. If the goal is to refresh the extrinsic config
before that, record a longer calibration bag with more overlap where the head
sees ID0+ID2 and the arm sees ID0; require at least `100` inliers.

Earlier Propagator guard/drop experiments were rejected because they could leave
the head node stuck dropping every camera update. The current 2026-05-14 retry is
narrower: keep RealSense/OpenVINS at `640x480x30`, cap only the internal stale
camera queue, and make incomplete IMU coverage a non-fatal `[PROP]` diagnostic
instead of a hard assertion. If the baseline still falls behind, the next step is
a Jetson-safe OpenVINS profile with fewer tracked features and a lower
`track_frequency`, not accepting intermittent VIO process death.

Recommended next-chat prompt:

```text
Read:
- .agents/AGENTS.md
- .agents/future_tasks.md
- .agents/phase2_openvins_handoff_status.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_marker_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dual_openvins_tf_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_marker_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_live_trial_and_recording.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_marker_pose_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py
- docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp
- docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.*

Make a detailed implementation plan for the next dynamic-ID2 feature: a
visualization/debug-only head-derived arm D435i pose preview. It should consume
`/head/marker_pose/dynamic_observation`, the head OpenVINS pose in `marker_map`,
and `config/markers/arm_marker_extrinsics.yaml` to compute a candidate
`T_map_armcam` when the head sees arm-mounted marker ID2. Preserve the validated
fixed-marker ID0 EKF path, do not add ID2 to `marker_fixed_ids`, and do not feed
this result into OpenVINS or update the arm state yet. Include message/topic
design, TF/RViz display options, covariance propagation, gating, validation on
`dual_openvins_id2_dynamic_phase2_20260511_194110`, and rollback steps.
```
