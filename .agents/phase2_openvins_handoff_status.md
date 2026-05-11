# Phase 2 OpenVINS Marker EKF Handoff Status

Use this note when starting a new Codex chat after the initial Phase 2
implementation work.

## Current Status

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
   ros2 pkg executables ov_msckf | grep run_subscribe_msckf_marker && \
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
- Python Phase 1 marker node now publishes:
  `/head/marker_pose/observation`
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
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_ekf_handoff_prompt.md`
- `docker_ws/multi_cam_localization/sensor_fusion_msgs/`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_marker_pose_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py`
- `docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`
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

Proceed from the no-RViz dual-camera checkpoint toward the dynamic arm-mounted
marker observation/calibration path. Before implementing online arm pose
updates, keep marker ID 2 out of the fixed `marker_map` update path and design a
separate dynamic-marker observation/calibration flow. The first calibration bag
should include simultaneous observations where both cameras see fixed marker ID
0 and the head camera also sees the arm-mounted marker ID 2.

The Propagator guard/drop experiment was rejected because it could leave the
head node stuck dropping every camera update. Current direction is to keep
RealSense/OpenVINS at `640x480x30`, reduce only the Python ArUco marker load
with `marker_detection_rate_hz:=15.0` and marker image queue depth 1, and avoid
long RViz/VS Code live runs when recording validation data.

Recommended next-chat prompt:

```text
Read:
- .agents/AGENTS.md
- .agents/future_tasks.md
- .agents/phase2_openvins_handoff_status.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_openvins_marker_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/phase2_dual_openvins_tf_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_marker_validation.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_phase2_openvins_live_trial_and_recording.md
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_d435i_openvins_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_d435i_openvins_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/head_marker_pose_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py
- docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py
- docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.cpp
- docker_ws/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h
- docker_ws/src/open_vins/ov_msckf/src/update/UpdaterMarkerPose.*

Plan a dynamic arm-mounted marker observation and calibration path for marker ID
2. Preserve the validated per-instance TF support and marker throttle. Marker ID
2 is mounted to the arm and must not be treated as a fixed marker_map landmark.
The calibration flow should collect samples when both cameras see fixed marker
ID 0 and the head camera also sees ID 2, estimate the fixed transform between ID
2 and the arm D435i, report residuals/uncertainty, and save the result to a
config file for later arm pose updates.
```
