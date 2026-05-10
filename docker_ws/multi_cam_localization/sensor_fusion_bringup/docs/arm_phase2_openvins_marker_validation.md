# Arm D435i Phase 2 OpenVINS Marker Validation

This recipe validates the arm D435i Phase 2 marker-enabled OpenVINS path in the
ROS 2 Jazzy Docker environment. Do not use host ROS commands for this workflow.

The arm setup mirrors the validated head Phase 2 path while keeping its ROS
topics separate:

```text
Camera:      /arm/d435i_arm/...
Marker node: /arm/marker_pose/...
OpenVINS:    /ov_msckf_arm/...
Map frame:   marker_map
```

The arm calibration source is:

```text
docker_ws/calibration/arm/d435i_310622071850/
```

The OpenVINS Propagator timing/crash guard is intentionally not part of this
bringup. Add that later unless the arm validation repeatedly hits the live
Propagator assertion and cannot produce a useful bag.

## Build

If the overlay is missing or stale, build in the Jazzy Docker container. Keep
OpenVINS builds sequential on the Jetson.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws
sudo chown -R "$(id -u):$(id -g)" build_overlay install_overlay log_overlay 2>/dev/null || true
rm -rf build_overlay install_overlay log_overlay
```

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm \
  -e MAKEFLAGS=-j1 \
  -e CMAKE_BUILD_PARALLEL_LEVEL=1 \
  realsense_camera \
  "source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && colcon --log-base log_overlay build \
    --executor sequential --parallel-workers 1 \
    --symlink-install \
    --build-base build_overlay \
    --install-base install_overlay \
    --packages-up-to sensor_fusion_bringup ov_msckf"
```

## Smoke Checks

Run this before live or replay validation:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 interface show sensor_fusion_msgs/msg/MarkerPoseObservation && \
   ros2 pkg executables ov_msckf | grep run_subscribe_msckf_marker && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py --show-args'
```

Expected:

- `run_subscribe_msckf_marker` is available.
- The arm OpenVINS launch exposes `verbosity`, `start_camera`, and
  `use_sim_time`.
- The arm marker launch exposes `config_file` and `use_sim_time`.
- No head launch or head topic is required for this arm-only smoke check.

## Config Checks

The arm config should resolve these inputs and outputs:

```text
/arm/d435i_arm/color/image_raw
/arm/d435i_arm/imu
/arm/marker_pose/observation
/ov_msckf_arm/poseimu
/ov_msckf_arm/odomimu
/ov_msckf_arm/pathimu
```

Marker observations must use:

```text
marker_id: 0
header.frame_id: marker_map
target_frame: arm_imu
```

The arm OpenVINS launch disables `publish_global_to_imu_tf` and
`publish_calibration_tf` to avoid duplicate `imu` and `cam0` TF frames when head
OpenVINS is also running. Use `/ov_msckf_arm/*` topics as the authoritative arm
OpenVINS outputs.

## Fresh Raw Replay Validation

Use this after recording a pre-init arm validation bag with
`arm_phase2_openvins_live_trial_and_recording.md`.

Terminal 1, start arm OpenVINS without the live camera:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py \
     start_camera:=false \
     use_sim_time:=true \
     verbosity:=INFO'
```

Terminal 2, start the arm marker node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py use_sim_time:=true'
```

Terminal 3, replay only raw arm inputs:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 bag play bags/openvins_tests/phase2_live/<arm_bag_name> --clock --topics \
     /arm/d435i_arm/color/image_raw \
     /arm/d435i_arm/color/camera_info \
     /arm/d435i_arm/imu'
```

Terminal 4, inspect regenerated output:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic list | grep -E "/ov_msckf_arm/(poseimu|odomimu|pathimu)|/arm/marker_pose/observation" && \
   ros2 topic echo --once /arm/marker_pose/observation && \
   ros2 topic echo --once /ov_msckf_arm/poseimu --field header && \
   ros2 topic echo --once /ov_msckf_arm/odomimu --field header && \
   ros2 topic echo --once /ov_msckf_arm/pathimu --field header'
```

## Acceptance Criteria

Replay passes when:

- OpenVINS initializes from raw arm image/IMU replay.
- `/arm/marker_pose/observation` publishes marker ID `0` observations.
- `/ov_msckf_arm/poseimu`, `/ov_msckf_arm/odomimu`, and
  `/ov_msckf_arm/pathimu` publish with `header.frame_id: marker_map`.
- Logs show marker-map first lock or relock from marker ID `0`.
- Logs show accepted marker ID `0` EKF updates after lock.
- Stale marker/timestamp drops are not the dominant behavior.
- The live Propagator assertion does not reproduce during replay.

Useful log strings:

```text
[MARKER]: reset OpenVINS global gauge to marker_map using marker 0 (first_marker_map_lock)
[MARKER]: accepted marker 0 EKF update (...)
[MARKER]: reset requested for marker 0 but velocity fit is not reliable yet (...)
```

## 2026-05-10 Arm Pre-Init Replay Result

Validated arm Phase 2 live bag:

```text
docker_ws/bags/openvins_tests/phase2_live/arm_marker_phase2_100mm_preinit_20260510_142201
```

Bag summary:

- Duration: `107.304767019 s`
- Size: `2.9 GiB`
- Raw input topics: `/arm/d435i_arm/color/image_raw` (`3059`),
  `/arm/d435i_arm/color/camera_info` (`3216`), `/arm/d435i_arm/imu`
  (`21424`)
- Recorded marker observations: `/arm/marker_pose/observation` (`1600`)
- Recorded OpenVINS outputs: `/ov_msckf_arm/poseimu` (`2586`),
  `/ov_msckf_arm/odomimu` (`13174`), `/ov_msckf_arm/pathimu` (`2586`)

Raw timestamp scan found monotonic image, camera-info, IMU, marker, and
OpenVINS output streams. Maximum observed header gaps were about `15 ms` for
IMU, `133 ms` for image, and `33 ms` for camera-info. Marker observations were
all marker ID `0`, all in `marker_map`, all target frame `arm_imu`, all
hard-gated true, and `1593 / 1600` stable.

Fresh replay used only raw arm input topics with `start_camera:=false` and
`use_sim_time:=true`. Replay passed:

- OpenVINS initialized and published arm pose/odom/path in `marker_map`.
- Marker ID `0` produced one first marker-map lock and `3` total marker-map
  resets.
- OpenVINS accepted `1664` marker ID `0` EKF updates.
- `16` reset requests were held until the marker-map velocity fit was reliable.
- No non-fixed marker rejections occurred.
- No fatal errors, segmentation faults, or hard Propagator assertion occurred.
- One replay warning was logged:
  `Propagator::select_imu_readings(): Missing inertial measurements to propagate
  with (-0.174664 sec missing)!`

The warning did not stop the replay and is not the live hard assertion observed
earlier on the head setup.

## Troubleshooting

- If OpenVINS never initializes, record or replay a bag that starts before the
  initialization jerk and includes a short stationary period first.
- If marker observations publish but OpenVINS rejects them, check that
  `target_frame` is `arm_imu` and the arm launch parameter
  `marker_target_frame` is also `arm_imu`.
- If the marker node reports no OpenVINS odom, check that
  `arm_aruco_map.yaml` uses `/ov_msckf_arm/odomimu`.
- If head and arm are both running, avoid relying on OpenVINS TF for the arm;
  validate arm OpenVINS through `/ov_msckf_arm/*` topics.
- OpenVINS still publishes `odomimu.child_frame_id: imu` internally. Arm TF is
  disabled until OpenVINS supports per-instance frame IDs or a safe TF prefix;
  otherwise head and arm TF trees would collide.
