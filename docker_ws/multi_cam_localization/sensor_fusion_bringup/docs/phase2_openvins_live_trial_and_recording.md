# Phase 2 OpenVINS Live Trial And Bag Recording

Use this when trying the marker-enabled OpenVINS path live with the final
100 mm x 100 mm marker. Use marker ID `0` only for the fixed map marker.

The Phase 2 marker launch defaults to `head_aruco_map.yaml`, where marker ID `0`
has `size_m: 0.100`. Do not use the `1533mm` or `160mm` marker configs for this
trial.

## 0. Check The Overlay

Run this first. If any line fails, rebuild with the low-memory command in
`phase2_openvins_marker_validation.md` before the live trial.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 interface show sensor_fusion_msgs/msg/MarkerPoseObservation && \
   ros2 pkg executables ov_msckf | grep run_subscribe_msckf_marker && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py --show-args'
```

## 1. Start Phase 2 OpenVINS

Terminal 1:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py verbosity:=INFO'
```

Expected behavior:

- The RealSense head camera starts.
- OpenVINS starts about 5 seconds later.
- Keep the camera stationary first.
- Do the normal acceleration jerk to initialize OpenVINS.
- After initialization, OpenVINS should publish `/ov_msckf/poseimu`,
  `/ov_msckf/odomimu`, and `/ov_msckf/pathimu` in `marker_map`.

Use `verbosity:=DEBUG` instead of `INFO` if you need to see stale/timestamp or
non-fixed-marker rejection details.

## 2. Start Phase 2 Marker Observations

Terminal 2:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py'
```

Expected behavior:

- The marker node uses `head_aruco_map.yaml`.
- The old external correction wrapper is disabled.
- `/head/marker_pose/observation` is published when marker ID `0` is visible and
  passes the marker gates.

## 3. Quick Topic Checks

Terminal 3:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic list | grep -E "/ov_msckf/(poseimu|odomimu|pathimu)|/head/marker_pose/observation"'
```

After marker ID `0` is visible:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic echo --once /head/marker_pose/observation'
```

Check that the observation says:

- `marker_id: 0`
- `header.frame_id: marker_map`
- `target_frame: head_imu`
- `hard_gate_passed: true`
- `stable: true`

Check OpenVINS output frames:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic echo --once /ov_msckf/poseimu --field header && \
   ros2 topic echo --once /ov_msckf/odomimu --field header && \
   ros2 topic echo --once /ov_msckf/odomimu --field child_frame_id && \
   ros2 topic echo --once /ov_msckf/pathimu --field header'
```

The frame ID should be `marker_map`, and `/ov_msckf/odomimu.child_frame_id`
should be `head_imu`.

## 4. Open RViz2

Use the head saved config for a head-only trial:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/head_openvins.rviz'
```

You do not need a new RViz config for the first live trial. The saved
`head_openvins.rviz` already uses `marker_map` as the fixed frame and already has
the important OpenVINS and marker-pose displays. For Phase 2, ignore or disable
the old `/head/marker_pose/ov_corrected_odom` display; that was the Phase 1
external correction output, and Phase 2 is now correcting inside OpenVINS.

For simultaneous head and arm trials, use:

```bash
rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/phase2_dual_openvins.rviz
```

Watch these frames:

- `marker_map`: RViz fixed frame.
- `marker_0`: fixed 100 mm marker map frame.
- `head_imu`: OpenVINS IMU frame after initialization and marker-map locking.
- `head_cam0`: OpenVINS camera calibration frame under `head_imu`.
- `head_imu_from_marker`: marker-only IMU pose estimate from the Python marker
  node.
- `head_d435i_head_color_optical_frame_body_display`: intuitive camera body
  display frame, where red `+X` points forward through the lens.

Watch these topics:

- `/ov_msckf/pathimu`: main Phase 2 OpenVINS path in `marker_map`.
- `/ov_msckf/odomimu`: current OpenVINS odometry in `marker_map`; enable this
  display if you want to see the current pose/covariance.
- `/head/marker_pose/camera_body_pose`: intuitive marker-derived camera pose.
- `/head/marker_pose/imu_pose`: marker-derived IMU pose in `marker_map`.
- `/head/marker_pose/camera_pose_raw`: raw optical camera pose; remember that
  optical `+Z` points through the lens, so the RViz arrow can look unintuitive.

Use terminal echoes for these non-visual status topics:

- `/head/marker_pose/observation`
- `/head/marker_pose/marker_valid`
- `/head/marker_pose/marker_quality`
- `/head/marker_pose/active_marker_id`

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
- Try marker ID `1` in view if you want. With the current 100 mm config, the
  marker node should ignore it; if an ID 1 observation reaches OpenVINS, OpenVINS
  should reject it as non-fixed.

Healthy behavior looks like:

```text
[MARKER]: accepted marker 0 EKF update (...)
```

Large corrections should reset only when the marker-map velocity fit is reliable:

```text
[MARKER]: reset requested for marker 0 but velocity fit is not reliable yet (...)
[MARKER]: reset OpenVINS global gauge to marker_map using marker 0 (...)
```

## 6. Record A Useful Validation Bag

The important detail is to start recording before the acceleration jerk that
initializes OpenVINS. Previous replay validation was limited because some bags
started after the live OpenVINS instance had already initialized.

Terminal 4, start this while the camera is stationary and before the jerk:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   mkdir -p bags/openvins_tests/phase2_live && \
   BAG=bags/openvins_tests/phase2_live/head_marker_phase2_100mm_preinit_$(date +%Y%m%d_%H%M%S) && \
   ros2 bag record -o "$BAG" \
     /head/d435i_head/color/image_raw \
     /head/d435i_head/color/camera_info \
     /head/d435i_head/imu \
     /head/marker_pose/observation \
     /head/marker_pose/active_marker_id \
     /head/marker_pose/marker_valid \
     /head/marker_pose/marker_quality \
     /head/marker_pose/imu_pose \
     /head/marker_pose/camera_pose_raw \
     /head/marker_pose/camera_body_pose \
     /ov_msckf/poseimu \
     /ov_msckf/odomimu \
     /ov_msckf/pathimu \
     /tf \
     /tf_static'
```

Recommended recording sequence:

1. Start Phase 2 OpenVINS and marker node.
2. Start the bag recorder while the camera is still stationary.
3. Record a few seconds stationary.
4. Do the acceleration jerk so OpenVINS initializes.
5. Show marker ID `0`.
6. Move gently with marker ID `0` visible.
7. Hide marker ID `0` briefly.
8. Show marker ID `0` again.
9. Optionally induce a larger VIO drift/reacquire case.
10. Stop recording with `Ctrl+C` so the bag closes cleanly.

For replay validation, launch OpenVINS with `start_camera:=false
use_sim_time:=true`, launch the marker node with `use_sim_time:=true`, then
replay only raw input topics:

```bash
ros2 bag play <bag_path> --clock --topics \
  /head/d435i_head/color/image_raw \
  /head/d435i_head/color/camera_info \
  /head/d435i_head/imu
```

The recorded marker/OpenVINS output topics are still useful for comparing what
happened live, but they should not be replayed into a fresh Phase 2 validation
run.

## 7. Recorded Live Bags

Confirmed useful Phase 2 live validation bag:

```text
docker_ws/bags/openvins_tests/phase2_live/head_marker_phase2_100mm_preinit_20260510_122723
```

Recorded on 2026-05-10 with the final 100 mm marker config. The recording starts
before OpenVINS initialization and includes multiple larger VIO drift/reacquire
events where marker ID `0` returned the estimate to the correct pose. The bag is
about 124.38 s long and 3.4 GiB.

Replay validation on 2026-05-10 used only raw replay topics:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/color/camera_info
/head/d435i_head/imu
```

Fresh Phase 2 replay result:

- OpenVINS initialized successfully.
- `/head/marker_pose/observation`: `879` marker ID `0` observations.
- `/ov_msckf/poseimu`: `1944` messages in `marker_map`.
- `/ov_msckf/odomimu`: `4310` messages in `marker_map`.
- `/ov_msckf/pathimu`: `1944` messages in `marker_map`.
- Marker ID `0` produced `4` marker-map resets and `1212` accepted EKF updates.
- `19` reset requests waited for a reliable marker-map velocity fit.
- No stale marker/timestamp drops were logged.
- The OpenVINS Propagator assertion seen once during live use did not reproduce
  during replay; OpenVINS was still running after bag playback.

Raw timestamp scan:

- IMU: monotonic, `24768` messages, max gap about `15 ms`.
- Image: monotonic, `3594` messages, some gaps up to about `100 ms`.
- Camera info: monotonic, `3729` messages, steady about `30 Hz`.

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
