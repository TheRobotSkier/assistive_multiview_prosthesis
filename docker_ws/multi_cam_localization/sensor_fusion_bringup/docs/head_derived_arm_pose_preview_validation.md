# Head-Derived Arm Pose Preview Validation

Use this recipe to launch and validate the debug-only head-derived arm D435i
pose preview. The preview consumes dynamic marker ID `2`, the head OpenVINS
pose, and `arm_marker_extrinsics.yaml`.

The preview does not feed OpenVINS, does not publish
`/arm/marker_pose/observation`, and does not add ID `2` to `marker_fixed_ids`.

Use Docker/Jazzy, not host ROS.

## 1. Build And Smoke Check

Run this after changing the preview node, launch file, RViz config, or tests:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && \
   colcon --log-base log_overlay build \
     --symlink-install \
     --build-base build_overlay \
     --install-base install_overlay \
     --packages-select sensor_fusion_bringup && \
   source install_overlay/setup.bash && \
   ros2 pkg executables sensor_fusion_bringup | grep head_derived_arm_pose_preview_node && \
   ros2 launch sensor_fusion_bringup head_derived_arm_pose_preview.launch.py --show-args'
```

Expected:
- `head_derived_arm_pose_preview_node.py` is listed as an executable.
- The launch arguments include `dynamic_observation_topic`, `head_pose_topic`,
  `output_prefix`, `marker_id`, `require_stable_dynamic_marker`, and
  `publish_tf`.

## 2. Run Tests

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   colcon --log-base log_overlay test \
     --build-base build_overlay \
     --install-base install_overlay \
     --packages-select sensor_fusion_bringup \
     --event-handlers console_direct+ && \
   colcon --log-base log_overlay test-result \
     --test-result-base build_overlay \
     --verbose'
```

Expected: `test_aruco_marker_pose_math` passes.

## 3. Replay Validation From Known Bag

This is the fastest test for the new preview node.

Terminal 1, start the preview node on replay time:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_derived_arm_pose_preview.launch.py use_sim_time:=true'
```

Terminal 2, replay the required recorded topics:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 bag play bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_20260511_194110 \
     --clock \
     --topics /head/marker_pose/dynamic_observation /ov_msckf/poseimu /tf /tf_static'
```

Terminal 3, check preview output:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   echo "preview pose frame:" && \
   ros2 topic echo --once /arm/marker_pose/head_derived/arm_camera_pose --field header.frame_id && \
   echo "preview status:" && \
   ros2 topic echo --once /arm/marker_pose/head_derived/status --field data'
```

Expected:

```text
/arm/marker_pose/head_derived/arm_camera_pose header.frame_id: marker_map
/arm/marker_pose/head_derived/status contains "accepted":true for good samples
```

Early replay samples can report `no_head_pose_match` before the bag reaches the
time window where dynamic ID2 observations and head OpenVINS poses overlap.
Keep the replay running until accepted samples appear.

## 4. Live Launch With Preview And RViz

For live validation, use the normal dual-stack launch commands from step 5 of
`phase2_dynamic_id2_marker_calibration_validation.md`, plus the preview launch
below. In practice this means six terminals.

Terminal 1, head OpenVINS:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py verbosity:=INFO'
```

Terminal 2, arm OpenVINS:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py verbosity:=INFO'
```

Terminal 3, head marker node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py marker_detection_rate_hz:=15.0'
```

Terminal 4, arm marker node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py marker_detection_rate_hz:=15.0'
```

Terminal 5, preview node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_derived_arm_pose_preview.launch.py'
```

Terminal 6, RViz2 with the preview config:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/phase2_dual_openvins_head_preview.rviz'
```

Live procedure:

1. Let both OpenVINS instances initialize.
2. Show fixed marker ID `0` to the head camera.
3. Show fixed marker ID `0` to the arm camera.
4. Show arm-mounted marker ID `2` to the head camera.
5. Move the arm gently while keeping ID2 visible to the head camera.

## 5. Topic Checks

Run this in another Docker/Jazzy shell after the live stack is running:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   echo "fixed/dynamic marker split:" && \
   ros2 topic echo --once /head/marker_pose/observation --field marker_id && \
   ros2 topic echo --once /head/marker_pose/dynamic_observation --field marker_id && \
   ros2 topic echo --once /arm/marker_pose/observation --field marker_id && \
   echo "preview pose frame:" && \
   ros2 topic echo --once /arm/marker_pose/head_derived/arm_camera_pose --field header.frame_id && \
   echo "preview status:" && \
   ros2 topic echo --once /arm/marker_pose/head_derived/status --field data'
```

Expected:

```text
/head/marker_pose/observation marker_id: 0
/head/marker_pose/dynamic_observation marker_id: 2
/arm/marker_pose/observation marker_id: 0
/arm/marker_pose/head_derived/arm_camera_pose header.frame_id: marker_map
/arm/marker_pose/head_derived/status contains "accepted":true
```

If the status reports `no_head_pose_match`, wait for head OpenVINS output and
try again. If it reports `dynamic_marker_not_stable`, keep ID2 in view until
the marker node reaches its stable-frame gate.

## 6. RViz Setup

The saved config already includes the preview displays. If setting RViz up
manually, use:

```text
Global Options / Fixed Frame: marker_map

TF:
  enabled
  show marker_map -> head_imu -> head_cam0
  show marker_map -> arm_imu -> arm_cam0
  show marker_map -> arm_d435i_arm_color_optical_frame_head_preview
  show marker_map -> arm_d435i_arm_color_optical_frame_head_preview_body_display
  show marker_map -> arm_marker_2_head_preview

Path:
  /ov_msckf/pathimu
  /ov_msckf_arm/pathimu
  /arm/marker_pose/head_derived/path

PoseWithCovariance:
  /arm/marker_pose/head_derived/arm_camera_pose

Pose:
  /arm/marker_pose/head_derived/arm_camera_body_pose
  /arm/marker_pose/head_derived/arm_marker_pose

Image:
  /ov_msckf/trackhist
  /ov_msckf_arm/trackhist
```

What to look for:
- The preview path appears only while the head sees dynamic marker ID2 and the
  head OpenVINS pose is available.
- The preview pose is visually near the real arm OpenVINS pose when the
  calibration and marker observation are good.
- Preview TF frames must use `_head_preview` suffixes and must not replace
  `arm_imu`, `arm_cam0`, or the real arm camera frames.
- Fixed observations remain ID0-only.

## 7. Validated Live Preview Bag

Validated on 2026-05-12:

```text
docker_ws/bags/openvins_tests/phase2_live/head_derived_arm_preview_20260512_073918
```

Bag metadata:

```text
duration: 67.12 s
size: 56.0 MiB
messages: 37943
```

Topic checks:

```text
/head/marker_pose/dynamic_observation: 162 messages, all marker ID 2
/arm/marker_pose/head_derived/status: 162 messages
/arm/marker_pose/head_derived/arm_camera_pose: 48 messages, frame marker_map
/arm/marker_pose/head_derived/arm_camera_body_pose: 48 messages, frame marker_map
/arm/marker_pose/head_derived/path: 48 messages
/ov_msckf/poseimu: 991 messages, frame marker_map
/ov_msckf_arm/poseimu: 1184 messages, frame marker_map
/tf: preview frames present with _head_preview suffixes
```

Preview status summary:

```text
accepted: 48
dynamic_marker_not_stable: 7
no_head_pose_match: 107
```

Dynamic marker quality:

```text
stable observations: 155 / 162
median reprojection error: 0.252 px
p95 reprojection error: 0.535 px
median distance: 0.582 m
p95 distance: 0.624 m
median view angle: 13.87 deg
p95 view angle: 16.95 deg
```

Preview covariance summary:

```text
median translational std: about 0.067-0.071 m
p95 translational std: about 0.075-0.081 m
median rotational std: about 5.84-5.96 deg
p95 rotational std: about 5.96-6.18 deg
```

Comparison against arm OpenVINS converted from `arm_imu` to arm camera frame:

```text
matched preview samples: 42 / 48
sync dt median: 0.0134 s
sync dt p95: 0.0450 s
translation residual median: 0.0646 m
translation residual rms: 0.0870 m
translation residual p95: 0.1978 m
rotation residual median: 2.06 deg
rotation residual rms: 2.44 deg
rotation residual p95: 3.69 deg
```

Interpretation: the preview path is working and the small visible offset is
expected. The residual is appropriate for debug visualization and planning the
next fusion step, but it should not be treated as ground truth. A future
OpenVINS dynamic-ID2 update must use covariance, timing, and innovation gates.

This bag is not a complete raw replay source. It did not record raw
camera/IMU streams or fixed marker observation topics. For future estimator
regression testing, use the full dynamic-ID2 bag
`dual_openvins_id2_dynamic_phase2_20260511_194110`, or record a new full bag
that includes raw head/arm inputs, fixed marker observations, dynamic
observations, OpenVINS output, preview output, `/tf`, and `/tf_static`.

## 8. Validated Full Raw Dynamic-ID2 Update Bag

Validated on 2026-05-12:

```text
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_full_raw_20260512_090704
```

Bag metadata:

```text
duration: 62.39 s
size: 2.9 GiB
messages: 71417
```

This is the preferred local bag for planning and replay-testing the future
dynamic ID2 arm OpenVINS update because it includes raw camera/IMU streams,
fixed marker observations, dynamic marker observations, existing OpenVINS
outputs, preview outputs, `/tf`, and `/tf_static`.

Topic checks:

```text
/head/d435i_head/color/image_raw: 1586 messages, rgb8, monotonic stamps
/head/d435i_head/color/camera_info: 1865 messages
/head/d435i_head/imu: 12178 messages, monotonic stamps
/arm/d435i_arm/color/image_raw: 1628 messages, rgb8, monotonic stamps
/arm/d435i_arm/color/camera_info: 1868 messages
/arm/d435i_arm/imu: 12196 messages, monotonic stamps
/head/marker_pose/observation: 525 messages, marker ID 0 only, target head_imu
/arm/marker_pose/observation: 302 messages, marker ID 0 only, target arm_imu
/head/marker_pose/dynamic_observation: 111 messages, marker ID 2 only
/arm/marker_pose/head_derived/arm_camera_pose: 18 messages, frame marker_map
/ov_msckf/poseimu: 1063 messages, frame marker_map
/ov_msckf_arm/poseimu: 996 messages, frame marker_map
/ov_msckf/odomimu.child_frame_id: head_imu
/ov_msckf_arm/odomimu.child_frame_id: arm_imu
/tf: expected head/arm TF chains and _head_preview frames present
```

Dynamic marker quality:

```text
stable observations: 104 / 111
median reprojection error: 0.274 px
p95 reprojection error: 0.604 px
median distance: 0.528 m
p95 distance: 0.569 m
median view angle: 12.78 deg
p95 view angle: 22.00 deg
```

Preview status summary:

```text
accepted: 18
dynamic_marker_not_stable: 7
no_head_pose_match: 86
```

Preview covariance summary:

```text
median translational std: about 0.061-0.070 m
p95 translational std: about 0.070-0.090 m
median rotational std: about 6.05-6.06 deg
p95 rotational std: about 6.34-6.82 deg
```

Comparison against arm OpenVINS converted from `arm_imu` to arm camera frame:

```text
matched preview samples: 17 / 18
sync dt median: 0.00265 s
sync dt p95: 0.03198 s
translation residual median: 1.023 m
translation residual rms: 11.52 m
translation residual p95: 22.11 m
rotation residual median: 2.97 deg
rotation residual rms: 3.47 deg
rotation residual p95: 5.59 deg
```

Interpretation: this bag is good for stressing and validating a future dynamic
ID2 arm-state update because the raw marker geometry is strong, while the
existing arm OpenVINS state can disagree substantially with the head-derived
arm-camera measurement. Do not use the current arm OpenVINS path in this bag as
ground truth.

Offline raw-image calibration characterization from this bag:

```text
Raw image counts: head 1586, arm 1628
Detections: head ID0 1578, head ID2 373, head ID0+ID2 373, arm ID0 822
Synchronized calibration samples: 159
Inliers/outliers: 154 / 5
Residuals: median translation 0.0055 m, p95 translation 0.0120 m,
           median rotation 1.827 deg, p95 rotation 3.839 deg
CameraInfo warnings: head 5.094 px, arm 3.863 px Kalibr mismatch
```

The calibration characterization exceeds the normal `100`-inlier gate, but it
was used only to characterize the bag. Do not overwrite the checked-in
`arm_marker_extrinsics.yaml` unless a separate calibration-refresh decision is
made.

Rosout warnings were limited to startup marker-node odom wait messages and one
arm marker-node `linear_velocity_implausible` VIO-health warning. No fatal
OpenVINS errors were recorded in `/rosout`.

## 9. Cleanup

Generated overlay artifacts should stay out of commits:

```text
docker_ws/build_overlay/
docker_ws/install_overlay/
docker_ws/log_overlay/
```

If Docker creates root/nobody-owned generated files, fix ownership only for
generated overlay directories from `docker_ws`:

```bash
sudo chown -R "$(id -u):$(id -g)" build_overlay install_overlay log_overlay
```
