# Arm D435i Phase 2 OpenVINS Live Trial And Bag Recording

Use this when trying the marker-enabled OpenVINS path live with the arm D435i.
Use marker ID `0` only for the fixed map marker. Do not add the moving
arm-mounted marker ID `1` as a fixed map marker in this Phase 2 validation.

The arm Phase 2 marker launch defaults to `arm_aruco_map.yaml`, where marker ID
`0` has `size_m: 0.100`.

## 0. Check The Overlay

Run this first. If any line fails, rebuild with the low-memory command in
`arm_phase2_openvins_marker_validation.md`.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 interface show sensor_fusion_msgs/msg/MarkerPoseObservation && \
   ros2 pkg executables ov_msckf | grep run_subscribe_msckf_marker && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py --show-args'
```

## 1. Start Arm Phase 2 OpenVINS

Terminal 1:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py verbosity:=INFO'
```

Expected behavior:

- The RealSense arm camera starts as `/arm/d435i_arm`.
- OpenVINS starts about 5 seconds later under `/ov_msckf_arm`.
- Keep the camera stationary first.
- Do the normal acceleration jerk to initialize OpenVINS.
- After initialization, OpenVINS should publish `/ov_msckf_arm/poseimu`,
  `/ov_msckf_arm/odomimu`, and `/ov_msckf_arm/pathimu` in `marker_map`.

Use `verbosity:=DEBUG` if you need marker rejection, stale/timestamp, or reset
details.

## 2. Start Arm Marker Observations

Terminal 2:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py'
```

Expected behavior:

- The marker node uses `arm_aruco_map.yaml`.
- The old external correction wrapper is disabled by launch override.
- `/arm/marker_pose/observation` publishes when marker ID `0` is visible and
  passes the marker gates.

## 3. Quick Topic Checks

Terminal 3:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic list | grep -E "/arm/d435i_arm|/arm/marker_pose|/ov_msckf_arm/(poseimu|odomimu|pathimu)"'
```

After marker ID `0` is visible:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic echo --once /arm/marker_pose/observation'
```

Check that the observation says:

- `marker_id: 0`
- `header.frame_id: marker_map`
- `target_frame: arm_imu`
- `hard_gate_passed: true`
- `stable: true`

Check OpenVINS output frames:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic echo --once /ov_msckf_arm/poseimu --field header && \
   ros2 topic echo --once /ov_msckf_arm/odomimu --field header && \
   ros2 topic echo --once /ov_msckf_arm/pathimu --field header'
```

The frame ID should be `marker_map`.

## 4. Optional RViz2

Use `marker_map` as the RViz fixed frame. The existing head RViz config is a
reasonable starting point, but its saved displays point at head topics. For the
arm trial, add or retarget displays to:

```text
/ov_msckf_arm/pathimu
/ov_msckf_arm/odomimu
/arm/marker_pose/camera_body_pose
/arm/marker_pose/imu_pose
/arm/marker_pose/camera_pose_raw
```

Arm OpenVINS calibration/global TF publishing is intentionally disabled to avoid
duplicate `imu` and `cam0` TF frames when the head setup is also running. The
arm marker node still publishes marker-derived visualization frames such as
`arm_imu_from_marker`.

This makes RViz less useful for judging drift while marker ID `0` is out of
view. Future work should add per-instance OpenVINS TF frame IDs or a safe TF
prefix so the arm can publish an arm-specific TF tree without colliding with the
head OpenVINS tree. The recorded `/ov_msckf_arm/odomimu` messages currently
still use `child_frame_id: imu`, so do not simply re-enable arm OpenVINS TF for
two-camera runs.

## 5. What To Try Live

Start simple:

1. Keep the camera stationary.
2. Initialize OpenVINS with the normal jerk.
3. Put marker ID `0` in view.
4. Watch Terminal 1 for a first marker-map lock:

```text
[MARKER]: reset OpenVINS global gauge to marker_map using marker 0 (first_marker_map_lock)
```

Then try:

- Gentle motion with marker ID `0` visible.
- Briefly hide marker ID `0`, then show it again.
- Let VIO drift a little, then bring marker ID `0` back.
- Try marker ID `1` only as a rejection check. It should not become a fixed
  marker-map landmark in this Phase 2 arm setup.

Healthy behavior looks like:

```text
[MARKER]: accepted marker 0 EKF update (...)
```

Large corrections should reset only when the marker-map velocity fit is
reliable:

```text
[MARKER]: reset requested for marker 0 but velocity fit is not reliable yet (...)
[MARKER]: reset OpenVINS global gauge to marker_map using marker 0 (...)
```

## 6. Record A Useful Validation Bag

Start recording before the acceleration jerk that initializes OpenVINS.

Terminal 4:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   mkdir -p bags/openvins_tests/phase2_live && \
   BAG=bags/openvins_tests/phase2_live/arm_marker_phase2_100mm_preinit_$(date +%Y%m%d_%H%M%S) && \
   ros2 bag record -o "$BAG" \
     /arm/d435i_arm/color/image_raw \
     /arm/d435i_arm/color/camera_info \
     /arm/d435i_arm/imu \
     /arm/marker_pose/observation \
     /arm/marker_pose/active_marker_id \
     /arm/marker_pose/marker_valid \
     /arm/marker_pose/marker_quality \
     /arm/marker_pose/imu_pose \
     /arm/marker_pose/camera_pose_raw \
     /arm/marker_pose/camera_body_pose \
     /ov_msckf_arm/poseimu \
     /ov_msckf_arm/odomimu \
     /ov_msckf_arm/pathimu \
     /tf \
     /tf_static \
     /rosout'
```

Recommended recording sequence:

1. Start arm Phase 2 OpenVINS and marker node.
2. Start the bag recorder while the camera is still stationary.
3. Record a few seconds stationary.
4. Do the acceleration jerk so OpenVINS initializes.
5. Show marker ID `0`.
6. Move gently with marker ID `0` visible.
7. Hide marker ID `0` briefly.
8. Show marker ID `0` again.
9. Optionally induce a larger VIO drift/reacquire case.
10. Stop recording with `Ctrl+C` so the bag closes cleanly.

For replay validation, replay only raw input topics:

```bash
ros2 bag play <bag_path> --clock --topics \
  /arm/d435i_arm/color/image_raw \
  /arm/d435i_arm/color/camera_info \
  /arm/d435i_arm/imu
```

The recorded marker/OpenVINS output topics are useful for comparison, but they
should not be replayed into a fresh Phase 2 validation run.

## 7. Recorded Live Bags

Confirmed useful arm Phase 2 live validation bag:

```text
docker_ws/bags/openvins_tests/phase2_live/arm_marker_phase2_100mm_preinit_20260510_142201
```

Recorded on 2026-05-10 with the final 100 mm marker config. The bag starts
before OpenVINS initialization and includes raw arm camera/IMU topics, regenerated
marker outputs, OpenVINS arm outputs, TF, TF static, and `/rosout`.

Fresh replay on 2026-05-10 used only raw input topics:

```text
/arm/d435i_arm/color/image_raw
/arm/d435i_arm/color/camera_info
/arm/d435i_arm/imu
```

Replay result:

- OpenVINS initialized and published `/ov_msckf_arm/poseimu`,
  `/ov_msckf_arm/odomimu`, and `/ov_msckf_arm/pathimu` in `marker_map`.
- `/arm/marker_pose/observation`: `1600` recorded observations, all marker ID
  `0`, all target frame `arm_imu`, all hard-gated true, `1593` stable.
- Fresh raw replay produced `3` marker-map resets and `1664` accepted marker ID
  `0` EKF updates.
- `16` reset requests waited for a reliable marker-map velocity fit.
- No fatal errors or hard Propagator assertion occurred during replay.

Raw timestamp scan:

- IMU: monotonic, `21424` messages, max gap about `15 ms`.
- Image: monotonic, `3059` messages, max gap about `133 ms`.
- Camera info: monotonic, `3216` messages, steady about `30 Hz`.

## 8. Stop Everything

Stop terminals with `Ctrl+C`. If a Docker terminal does not exit cleanly:

```bash
docker ps
docker stop <container_id>
```

Before committing source changes, make sure generated overlays are not in
`git status`:

```bash
cd ~/Documents/assistive_multiview_prosthesis
git status --short docker_ws/build_overlay docker_ws/install_overlay docker_ws/log_overlay
```
