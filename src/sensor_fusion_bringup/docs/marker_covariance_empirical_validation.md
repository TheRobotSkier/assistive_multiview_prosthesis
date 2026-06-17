# Marker Covariance Empirical Validation

Use this procedure after the Phase 1 covariance code builds and before treating
the covariance model as ready for Phase 2 OpenVINS-internal marker updates.

Run ROS 2 commands inside the Jazzy `realsense_camera` Docker container.

## Purpose

Use two different recording types:

- Estimation bags: three stationary 100 mm markers, IDs 0, 1, and 2, in the
  same image with mixed distances/orientations. These bags provide more samples
  per recording for fitting or tuning covariance behavior.
- Validation bags: the final fixed reference marker ID 0, 100 mm, in the
  expected head-camera layout. These bags confirm the tuned model behaves in
  the real Phase 1 setup.

The normal Phase 1 correction node still uses fixed marker ID 0 only. The
three-marker estimation data is collected by `marker_quality_monitor.py`, which
detects all visible markers independently and publishes `/head/marker_pose/all_marker_quality`.

## Marker Setup

Use the final printed 6x6 markers:

```text
Marker IDs for estimation: 0, 1, 2
Physical marker size: 100 mm x 100 mm
Default head config: multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
Configured size_m: 0.100
```

Do not use the old May 2026 replay config for new covariance validation. That
config is only for the old 153.3 mm printed marker used in those recordings.

For estimation, place the three markers close enough that the D435i can see all
of them, but give them different positions and orientations. They do not need
to be at exact measured distances or angles. The monitor estimates distance and
view angle per marker from each image.

## Recommended Bag Directory

```text
/miahand_ws/src/bags/openvins_tests/head_marker_covariance
```

## Command Order

For a full live validation rerun, run the commands in this section exactly in
terminal order:

```text
Terminal 1: Start Live Head Stack
Terminal 2: Start Phase 1 Marker Correction Node
Terminal 3: Start All-Marker Quality Monitor
Terminal 4: Sanity Check Topics, then record the final validation bags
```

Keep Terminals 1-3 running while each final validation bag records. Stop each
`ros2 bag record` with `Ctrl+C` so the MCAP closes cleanly.

For Phase 1 closeout after the 2026-05-07 bag analysis, a full rerun is not
required. A live sanity check is sufficient if it confirms the expected Phase 1
boundary: external corrected odom snaps back to marker ID 0 after VIO drift, but
the internal OpenVINS state, uncertainty, and velocity remain uncorrected.

### 1. Start Live Head Stack

Terminal 1:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose stop realsense_camera 2>/dev/null || true
docker rm -f marker_cov_live 2>/dev/null || true

docker compose run --rm --name marker_cov_live --service-ports --entrypoint /bin/bash realsense_camera -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 launch sensor_fusion_bringup head_d435i_openvins.launch.py \
    rviz_enable:=false \
    verbosity:=INFO
'
```

### 2. Start Phase 1 Marker Correction Node

Terminal 2:

```bash
docker exec -it marker_cov_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 launch sensor_fusion_bringup head_marker_pose.launch.py
'
```

### 3. Start All-Marker Quality Monitor

Terminal 3:

```bash
docker exec -it marker_cov_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 run sensor_fusion_bringup marker_quality_monitor.py \
    --ros-args \
    -p marker_size_m:=0.100 \
    -p marker_ids:=0,1,2 \
    -p print_hz:=2.0
'
```

The monitor prints compact live rows like:

```text
id=0 dist=0.742m angle=28.6deg side=91.2px reproj=0.412px | id=1 ...
```

It also publishes baggable JSON on:

```text
/head/marker_pose/all_marker_quality
```

### 4. Sanity Check Topics

Terminal 4:

```bash
docker exec -it marker_cov_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 topic echo --once /head/marker_pose/all_marker_quality
  ros2 topic echo --once /head/marker_pose/marker_quality
  ros2 topic echo --once /head/marker_pose/marker_valid
'
```

## Estimation Bags: Three-Marker Stationary Mixed Layout

Record roughly 15 seconds per setup. Keep camera and markers stationary while
each bag records, then stop with `Ctrl+C` so `ros2 bag` closes the bag cleanly.
Move to a new approximate setup before starting the next bag.

Recommended setups:

```text
near_mixed:      about 0.4-0.5 m
medium_mixed:    about 0.75-1.0 m
far_mixed:       about 1.25-1.5 m
oblique_mixed:   about 45-60 deg marker/camera angle, optional if mixed bags already cover oblique views
difficult_mixed: near border or high obliqueness while still mostly visible, optional stress case
```

Use a descriptive `SETUP` name for each recording:

```bash
docker exec -it marker_cov_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  mkdir -p /miahand_ws/src/bags/openvins_tests/head_marker_covariance
  cd /miahand_ws/src/bags/openvins_tests/head_marker_covariance

  SETUP=near_mixed
  BAG=head_marker_cov_estimation_100mm_${SETUP}_$(date +%Y%m%d_%H%M%S)

  ros2 bag record -o "$BAG" --topics \
    /head/d435i_head/color/image_raw \
    /head/d435i_head/color/camera_info \
    /head/marker_pose/all_marker_quality \
    /head/marker_pose/marker_quality \
    /head/marker_pose/marker_valid \
    /head/marker_pose/active_marker_id \
    /head/marker_pose/reanchor_event \
    /rosout
'
```

Repeat the command after changing `SETUP` and moving the camera/markers to the
next approximate setup. For this validation run, multiple recordings were made
for `near_mixed`, `medium_mixed`, and `far_mixed`; `oblique_mixed` and
`difficult_mixed` were not recorded because the mixed near/medium/far bags were
judged sufficient for the first empirical pass.

Manual `Ctrl+C` is preferred for these recordings. Jazzy `ros2 bag record` does
not have a true stop-after-duration option; `--max-bag-duration` only splits
bag files and does not stop the recorder.

## Validation Bags: Final Single-Marker Layout

After the estimation bags, validate with the final fixed reference marker ID 0
in the expected user-facing layout. This does not need the full estimation grid.
These recordings are useful when the full validation sequence is needed, but
they do not need to be repeated for every Phase 1 closeout if the prior bag
analysis is already reviewed and the live sanity check passes.

Recommended validation recordings:

```text
final_stationary: 15-30 s, final marker layout, camera still
final_gentle_motion: slow head-camera motion
final_lost_regained_vio_good: marker moves out of view and returns while VIO stays healthy
final_lost_regained_vio_drift: marker moves out of view and returns after VIO tracking degrades
```

Use the same command for each validation recording and change only `SETUP`:

```bash
docker exec -it marker_cov_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  mkdir -p /miahand_ws/src/bags/openvins_tests/head_marker_covariance
  cd /miahand_ws/src/bags/openvins_tests/head_marker_covariance

  SETUP=final_stationary
  BAG=head_marker_cov_validation_100mm_${SETUP}_$(date +%Y%m%d_%H%M%S)

  ros2 bag record -o "$BAG" --topics \
    /head/d435i_head/color/image_raw \
    /head/d435i_head/color/camera_info \
    /head/d435i_head/imu \
    /ov_msckf/odomimu \
    /ov_msckf/poseimu \
    /ov_msckf/pathimu \
    /head/marker_pose/all_marker_quality \
    /head/marker_pose/active_marker_id \
    /head/marker_pose/camera_pose_raw \
    /head/marker_pose/camera_body_pose \
    /head/marker_pose/imu_pose \
    /head/marker_pose/marker_valid \
    /head/marker_pose/marker_quality \
    /head/marker_pose/vio_valid \
    /head/marker_pose/ov_corrected_odom \
    /head/marker_pose/reanchor_event \
    /tf \
    /tf_static \
    /rosout
'
```

Use these `SETUP` values:

```text
final_stationary
final_gentle_motion
final_lost_regained_vio_good
final_lost_regained_vio_drift
```

## Recorded Bag Inventory

Recorded on 2026-05-07 in:

```text
docker_ws/bags/openvins_tests/head_marker_covariance
```

Estimation bags:

```text
head_marker_cov_estimation_100mm_near_mixed_20260507_121512
head_marker_cov_estimation_100mm_near_mixed_20260507_122127
head_marker_cov_estimation_100mm_near_mixed_20260507_122528
head_marker_cov_estimation_100mm_medium_mixed_20260507_124034
head_marker_cov_estimation_100mm_medium_mixed_20260507_124313
head_marker_cov_estimation_100mm_medium_mixed_20260507_124702
head_marker_cov_estimation_100mm_far_mixed_20260507_125839
head_marker_cov_estimation_100mm_far_mixed_20260507_130210
head_marker_cov_estimation_100mm_far_mixed_20260507_130656
```

Validation bags:

```text
head_marker_cov_validation_100mm_final_stationary_20260507_132957
head_marker_cov_validation_100mm_final_gentle_motion_20260507_135934
head_marker_cov_validation_100mm_final_lost_regained_vio_good_20260507_140742
head_marker_cov_validation_100mm_final_lost_regained_vio_drift_20260507_141607
```

All 13 bag directories contain `metadata.yaml`, an `.mcap` file, and the color
image topic. The 9 estimation bags and 3 of the 4 validation bags contain
messages on `/head/marker_pose/all_marker_quality`. The
`final_stationary` validation bag lists the topic but contains 0 messages, so
use `/head/marker_pose/marker_quality` and `/head/marker_pose/imu_pose` for
that bag. All 4 validation bags contain `/ov_msckf/odomimu` and
`/head/marker_pose/ov_corrected_odom`.

## Analyze Recorded Bags

The repeatable analysis tool is documented in
`marker_covariance_bag_analysis.md`.

Run it inside the Jazzy `realsense_camera` container after rebuilding the
overlay:

```bash
ros2 run sensor_fusion_bringup analyze_marker_covariance_bags.py \
  --bag-root /miahand_ws/src/bags/openvins_tests/head_marker_covariance \
  --config /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
```

For a source-tree run before rebuilding the overlay:

```bash
python3 /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/scripts/analyze_marker_covariance_bags.py \
  --bag-root /miahand_ws/src/bags/openvins_tests/head_marker_covariance \
  --config /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
```

The default output directory is:

```text
/miahand_ws/src/bags/openvins_tests/head_marker_covariance/analysis_phase1_marker_covariance
```

## What To Inspect

For estimation bags, use `/head/marker_pose/all_marker_quality` because it logs
every visible marker independently.

For validation bags, use both:

- `/head/marker_pose/all_marker_quality` for all visible marker measurements
- `/head/marker_pose/marker_quality` and `/head/marker_pose/ov_corrected_odom`
  for the actual Phase 1 correction output

Acceptance targets:

- covariance increases with distance, view angle, reprojection error, and small
  marker pixel size
- stationary pose jitter is usually inside the predicted standard deviations
- bad detections are rejected before covariance weighting
- corrected odom still reanchors with the final 100 mm config

Phase 1 covariance should be considered empirical-baseline-ready after the
recorded 100 mm bags are reviewed and a live sanity check confirms the expected
external-correction behavior.

## Phase 1 Closeout Criteria

Before starting Phase 2 OpenVINS-internal EKF marker work, confirm through the
existing analysis plus live observation:

- corrected odom remains stable when the marker and camera are stationary
- marker pose estimates still look physically correct for the final marker ID 0
- after VIO drift and marker reacquisition, `/head/marker_pose/ov_corrected_odom`
  snaps back to the marker-consistent pose
- the remaining drift, growing uncertainty, and velocity growth are understood
  as Phase 1 external-correction limitations, not covariance-model failures
- `analysis_report.md` recommends no urgent covariance config change

One optional short evidence bag can be recorded for future comparison with a
setup name such as `final_vio_drift_recovery_phase1_limit`, but it is not
required before committing Phase 1. After closeout, commit and push the Phase 1
baseline before opening the Phase 2 OpenVINS EKF implementation task.
