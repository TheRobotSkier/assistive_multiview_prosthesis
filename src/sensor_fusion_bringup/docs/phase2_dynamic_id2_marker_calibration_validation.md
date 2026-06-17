# Phase 2 Dynamic ID2 Marker Calibration Validation

Use this recipe to validate the dynamic arm-mounted marker ID `2` path.
Marker ID `2` must stay out of the fixed marker-map EKF update path. The fixed
marker ID `0` remains the only shared `marker_map` reference for head and arm
OpenVINS.

For the shorter future-facing command sheet used when a new physical ID2 mount
changes the arm-camera transform, see
`arm_marker_id2_extrinsic_calibration_commands.md`.

Current accepted arm-marker extrinsic: the replacement ID2 mount was calibrated
on 2026-05-21 from
`dynamic_id2_arm_update_live_20260521_144725` and installed in
`config/markers/arm_marker_extrinsics.yaml`. That calibration produced `561`
inliers, p95 translation residual `0.0118 m`, and p95 rotation residual
`3.888 deg`. Older bags/results below are retained as historical validation
evidence and should not be treated as the current checked-in extrinsic.

Expected split:

```text
/head/marker_pose/observation          fixed marker observations, marker_id 0 only
/arm/marker_pose/observation           fixed marker observations, marker_id 0 only
/head/marker_pose/dynamic_observation  dynamic arm marker observations, marker_id 2
```

The dynamic observation pose is `T_head_camera_marker2` in the head camera
optical frame. It is not a `marker_map` pose and is not consumed by OpenVINS.
RViz2 does not display this custom message type directly; inspect it with
`ros2 topic echo` from a Docker/Jazzy shell that has sourced
`install_overlay/setup.bash`.

Known calibration-source bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816
```

Expected offline calibration checkpoint from that bag:

```text
Raw image counts: head 1567, arm 1457
Detections: head ID0 468, head ID2 1035, head ID0+ID2 259, arm ID0 1457
Synchronized calibration samples: 230
Inliers/outliers: 173 / 57
Residuals: median translation 0.0130 m, p95 translation 0.0273 m,
           median rotation 1.801 deg, p95 rotation 3.541 deg
CameraInfo warnings: head 5.094 px, arm 3.863 px Kalibr mismatch
```

Latest live dynamic-ID2 validation bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_20260511_194110
```

Read-only validation result:
- Bag metadata: `92.26 s`, `4.3 GiB`, `110360` messages.
- `/head/marker_pose/dynamic_observation`: `320` messages, marker ID `2`
  only, `camera_frame: head_d435i_head_color_optical_frame`,
  `marker_frame: arm_marker_2`.
- `/head/marker_pose/observation`: `763` messages, marker ID `0` only,
  `target_frame: head_imu`.
- `/arm/marker_pose/observation`: `367` messages, marker ID `0` only,
  `target_frame: arm_imu`.
- `/ov_msckf/odomimu.child_frame_id`: `head_imu`.
- `/ov_msckf_arm/odomimu.child_frame_id`: `arm_imu`.
- TF contains the expected `marker_map -> head_imu -> head_cam0` and
  `marker_map -> arm_imu -> arm_cam0` chains.

Calibration result from this live bag:
- raw detections: head ID0 `2062`, head ID2 `837`, head ID0+ID2 `759`, arm ID0
  `990`
- synchronized calibration samples: `72`
- strict acceptance with `--min-inliers 100` failed because only `68` inliers
  were available
- lower-threshold characterization with `--min-inliers 50` produced `68`
  inliers, `4` outliers, p95 translation residual `0.0252 m`, and p95 rotation
  residual `2.067 deg`

This bag is good evidence that the runtime fixed/dynamic topic split works. It
is not strong enough to replace the checked-in calibration config because the
arm camera did not see fixed marker ID `0` often enough while the head camera
saw ID0+ID2.

Latest full raw dynamic-ID2 update planning bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_full_raw_20260512_090704
```

Read-only validation result:
- Bag metadata: `62.39 s`, `2.9 GiB`, `71417` messages.
- Raw streams are present for both cameras: head/arm color image, camera info,
  and IMU topics, with monotonic image and IMU stamps.
- `/head/marker_pose/observation`: `525` messages, marker ID `0` only,
  `target_frame: head_imu`.
- `/arm/marker_pose/observation`: `302` messages, marker ID `0` only,
  `target_frame: arm_imu`.
- `/head/marker_pose/dynamic_observation`: `111` messages, marker ID `2`
  only, `camera_frame: head_d435i_head_color_optical_frame`,
  `marker_frame: arm_marker_2`.
- `/arm/marker_pose/head_derived/arm_camera_pose`: `18` accepted preview
  messages in `marker_map`.
- OpenVINS outputs remain separated: head pose in `marker_map`,
  arm pose in `marker_map`, odom child frames `head_imu` and `arm_imu`.
- TF contains the expected head/arm chains and `_head_preview` debug frames.

Calibration characterization from this full raw bag:
- raw detections: head ID0 `1578`, head ID2 `373`, head ID0+ID2 `373`, arm ID0
  `822`
- synchronized calibration samples: `159`
- inliers/outliers: `154 / 5`
- p95 translation residual `0.0120 m`
- p95 rotation residual `3.839 deg`

This is the preferred local source bag for planning/replay-testing a future
dynamic-ID2 arm update. It should still not overwrite the checked-in calibration
without a separate calibration-refresh decision.

## 0. Before Starting

Use Docker/Jazzy, not host ROS. The host may have a different ROS distro.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
docker ps
```

If stale validation containers are running, stop only those stale containers:

```bash
docker stop <container_id>
```

Avoid long RViz runs while recording if CPU load is high. The previous live
OpenVINS timing failure was load-sensitive, so a short RViz visual check plus a
no-RViz recording is usually safer.

These warnings are not validation failures by themselves:
- RViz `Stereo is NOT SUPPORTED`
- `XDG_RUNTIME_DIR not set`
- CycloneDDS `Failed to parse type hash ... USER_DATA '(null)'`

Treat them as benign if the bag subscribes to the requested topics and the
topic/message checks below pass.

## 1. Build And Smoke Check

Run this after changing the message, marker node, or calibration script:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && \
   colcon --log-base log_overlay build \
     --symlink-install \
     --build-base build_overlay \
     --install-base install_overlay \
     --packages-select sensor_fusion_msgs sensor_fusion_bringup && \
   source install_overlay/setup.bash && \
   ros2 interface show sensor_fusion_msgs/msg/DynamicMarkerObservation && \
   ros2 pkg executables sensor_fusion_bringup | grep calibrate_arm_marker_extrinsic && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py --show-args'
```

Expected:
- `DynamicMarkerObservation` exists.
- `calibrate_arm_marker_extrinsic.py` is installed as an executable.
- Head marker launch still exposes `config_file`, `use_sim_time`, and
  `marker_detection_rate_hz`.
- Head and arm OpenVINS launches still use `marker_fixed_ids: "0"` in source.

Confirm the fixed/dynamic config split:

```bash
cd ~/Documents/assistive_multiview_prosthesis

grep -RInE "marker_fixed_ids|dynamic_markers|dynamic_observation" \
  docker_ws/multi_cam_localization/sensor_fusion_bringup/launch \
  docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers
```

Expected:

```text
head_aruco_map.yaml has dynamic_observation_topic_suffix and dynamic_markers
head_d435i_openvins_phase2.launch.py has marker_fixed_ids: "0"
arm_d435i_openvins_phase2.launch.py has marker_fixed_ids: "0"
```

## 2. Run Unit Tests

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

## 3. Offline Calibration From The Known ID2 Bag

This validates the raw-image calibration path without launching live nodes. It
writes to `/tmp` so the checked-in calibration is not overwritten during a
routine validation pass.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rm -rf /tmp/arm_marker_extrinsic_validation_report && \
   ros2 run sensor_fusion_bringup calibrate_arm_marker_extrinsic.py \
     --bag bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816 \
     --head-config multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml \
     --arm-config multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml \
     --dynamic-marker-id 2 \
     --output /tmp/arm_marker_extrinsics_validation.yaml \
     --report-dir /tmp/arm_marker_extrinsic_validation_report \
     --min-inliers 100 && \
   sed -n "1,220p" /tmp/arm_marker_extrinsics_validation.yaml && \
   ls -lh /tmp/arm_marker_extrinsic_validation_report && \
   head -5 /tmp/arm_marker_extrinsic_validation_report/arm_marker_extrinsic_residuals.csv'
```

Expected:
- At least `100` inliers.
- About `230` synchronized samples on the known bag.
- Residuals roughly in the checkpoint range above.
- CameraInfo/Kalibr mismatch warnings may appear; they are expected for the
  current known bag unless `--strict-camera-info` is used.

The calibration and inspection are intentionally in the same Docker command.
Each `docker compose run --rm ...` starts a fresh container, so files written to
that container's `/tmp` are gone when a later disposable container starts.

To intentionally refresh the checked-in calibration file, use the same command
but set:

```text
--output multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
```

Only commit the refreshed config after reviewing residuals.

## 4. Replay Dynamic Observation Topic From The Known Bag

This confirms that replayed raw head images generate ID2 on the dynamic topic
while fixed observations remain ID0.

Terminal 1, run the head marker node on replay time:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py \
     use_sim_time:=true \
     marker_detection_rate_hz:=15.0'
```

Terminal 2, replay only the head raw image stream slowly:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 bag play bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816 \
     --clock \
     --rate 0.5 \
     --topics /head/d435i_head/color/image_raw'
```

Terminal 3, sample the topics:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   echo "dynamic marker id:" && \
   ros2 topic echo --once /head/marker_pose/dynamic_observation --field marker_id && \
   echo "dynamic frame:" && \
   ros2 topic echo --once /head/marker_pose/dynamic_observation --field header.frame_id && \
   echo "fixed marker id:" && \
   ros2 topic echo --once /head/marker_pose/observation --field marker_id'
```

Expected:

```text
/head/marker_pose/dynamic_observation marker_id: 2
/head/marker_pose/dynamic_observation header.frame_id: head_d435i_head_color_optical_frame
/head/marker_pose/observation marker_id: 0
```

If `ros2 topic info` can see `sensor_fusion_msgs/msg/DynamicMarkerObservation`
but `ros2 topic echo` says the message type is invalid, the echo command is
running in a shell that has not sourced the generated overlay. Use the Docker
commands above, or source `/miahand_ws/src/install_overlay/setup.bash` in the
same shell before echoing the custom message.

Stop Terminals 1 and 2 with `Ctrl+C`.

## 5. Live Dual-Camera Validation With RViz

Start the full dual stack. Keep the normal RealSense/OpenVINS streams at
`640x480x30`; only the Python marker detector is throttled to `15 Hz`.

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

Terminal 5, RViz2 with the saved dual config:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/phase2_dual_openvins.rviz'
```

Live procedure:

1. Let both OpenVINS instances initialize.
2. Show fixed marker ID `0` to the head camera and arm camera.
3. Show arm-mounted marker ID `2` to the head camera.
4. Confirm RViz fixed frame is `marker_map`.
5. Confirm the TF trees remain:

```text
marker_map -> head_imu -> head_cam0
marker_map -> arm_imu  -> arm_cam0
```

Terminal 6, topic checks:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   echo "topics:" && \
   ros2 topic list | grep -E "/head/marker_pose/(observation|dynamic_observation)|/arm/marker_pose/observation|/ov_msckf(_arm)?/(poseimu|odomimu|pathimu)" && \
   echo "head fixed marker id:" && \
   ros2 topic echo --once /head/marker_pose/observation --field marker_id && \
   echo "head dynamic marker id:" && \
   ros2 topic echo --once /head/marker_pose/dynamic_observation --field marker_id && \
   echo "head dynamic camera frame:" && \
   ros2 topic echo --once /head/marker_pose/dynamic_observation --field camera_frame && \
   echo "arm fixed marker id:" && \
   ros2 topic echo --once /arm/marker_pose/observation --field marker_id'
```

Expected:

```text
head fixed marker id: 0
head dynamic marker id: 2
head dynamic camera frame: head_d435i_head_color_optical_frame
arm fixed marker id: 0
```

If this command is run outside the Docker/Jazzy overlay, `ros2 topic list` and
`ros2 topic info` may see the topic over DDS but `ros2 topic echo` may fail with
`The message type 'sensor_fusion_msgs/msg/DynamicMarkerObservation' is invalid`.
Run the check through the Docker command shown above, or use a shell where
`source /opt/ros/jazzy/setup.bash` and `source install_overlay/setup.bash` have
both been run.

Seeing `/head/marker_pose/dynamic_observation` confirms ID2 detection only. It
does not mean the arm D435i OpenVINS state will update from ID2 yet; online arm
pose/state fusion is intentionally not implemented in this phase.

## 6. Debug-Only Head-Derived Arm Pose Preview

The preview node consumes dynamic ID2 observations, the head OpenVINS pose, and
`arm_marker_extrinsics.yaml` to publish a candidate arm D435i camera pose for
RViz/debugging. It does not publish `/arm/marker_pose/observation`, does not
change `marker_fixed_ids`, and does not feed OpenVINS.

Replay the latest live dynamic-ID2 bag and start the preview node:

Terminal 1:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_derived_arm_pose_preview.launch.py use_sim_time:=true'
```

Terminal 2:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 bag play bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_20260511_194110 \
     --clock \
     --topics /head/marker_pose/dynamic_observation /ov_msckf/poseimu /tf /tf_static'
```

Terminal 3:

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
/arm/marker_pose/head_derived/status contains accepted=true for good samples
```

Optional RViz display:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/phase2_dual_openvins_head_preview.rviz'
```

RViz should show the preview topics and TF frames with `_head_preview` suffixes:

```text
/arm/marker_pose/head_derived/arm_camera_pose
/arm/marker_pose/head_derived/arm_camera_body_pose
/arm/marker_pose/head_derived/path
marker_map -> arm_d435i_arm_color_optical_frame_head_preview
marker_map -> arm_d435i_arm_color_optical_frame_head_preview_body_display
marker_map -> arm_marker_2_head_preview
```

The preview path is a candidate camera pose only. Do not use it as acceptance
evidence for online arm-state updates.

## 7. Record A Live Dynamic-ID2 Validation Bag

For the most useful bag, start recording while both cameras are stationary and
before the OpenVINS initialization jerks. If RViz makes the system sluggish,
close RViz before a longer recording and rely on the bag for later replay.

Terminal 7:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   mkdir -p bags/openvins_tests/phase2_live && \
   BAG=bags/openvins_tests/phase2_live/dual_openvins_id2_dynamic_phase2_$(date +%Y%m%d_%H%M%S) && \
   echo "Recording $BAG" && \
   ros2 bag record -o "$BAG" --topics \
     /head/d435i_head/color/image_raw \
     /head/d435i_head/color/camera_info \
     /head/d435i_head/imu \
     /arm/d435i_arm/color/image_raw \
     /arm/d435i_arm/color/camera_info \
     /arm/d435i_arm/imu \
     /head/marker_pose/observation \
     /head/marker_pose/dynamic_observation \
     /head/marker_pose/active_marker_id \
     /head/marker_pose/marker_valid \
     /head/marker_pose/marker_quality \
     /head/marker_pose/imu_pose \
     /head/marker_pose/camera_pose_raw \
     /head/marker_pose/camera_body_pose \
     /arm/marker_pose/observation \
     /arm/marker_pose/active_marker_id \
     /arm/marker_pose/marker_valid \
     /arm/marker_pose/marker_quality \
     /arm/marker_pose/imu_pose \
     /arm/marker_pose/camera_pose_raw \
     /arm/marker_pose/camera_body_pose \
     /ov_msckf/poseimu \
     /ov_msckf/odomimu \
     /ov_msckf/pathimu \
     /ov_msckf_arm/poseimu \
     /ov_msckf_arm/odomimu \
     /ov_msckf_arm/pathimu \
     /tf \
     /tf_static \
     /rosout'
```

Recommended recording sequence:

1. Start both OpenVINS launches and both marker launches.
2. Start the recorder while both cameras are stationary.
3. Record a few seconds stationary.
4. Initialize head OpenVINS.
5. Initialize arm OpenVINS.
6. Show marker ID `0` to the head camera.
7. Show marker ID `0` to the arm camera.
8. Show arm-mounted marker ID `2` to the head camera.
9. Move the arm gently through several poses while keeping ID2 visible to the
   head and ID0 visible to both cameras when possible.
10. Stop the recorder with `Ctrl+C` so the MCAP closes cleanly.

## 8. Validate A Newly Recorded Bag

Replace `<bag_name>` with the directory printed by the recorder.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 bag info bags/openvins_tests/phase2_live/<bag_name>'
```

Expected topics include:

```text
/head/marker_pose/dynamic_observation
/head/marker_pose/observation
/arm/marker_pose/observation
/ov_msckf/poseimu
/ov_msckf_arm/poseimu
/tf
/tf_static
```

Run offline calibration on the new bag:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rm -rf /tmp/arm_marker_extrinsic_new_bag_report && \
   ros2 run sensor_fusion_bringup calibrate_arm_marker_extrinsic.py \
     --bag bags/openvins_tests/phase2_live/<bag_name> \
     --head-config multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml \
     --arm-config multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml \
     --dynamic-marker-id 2 \
     --output /tmp/arm_marker_extrinsics_new_bag.yaml \
     --report-dir /tmp/arm_marker_extrinsic_new_bag_report \
     --min-inliers 100'
```

Acceptance criteria for a good new bag:
- `/head/marker_pose/dynamic_observation` is present and reports marker ID `2`.
- `/head/marker_pose/observation` and `/arm/marker_pose/observation` report
  marker ID `0`, not ID `2`.
- The calibration script gets at least `100` inliers.
- p95 translation residual is reasonably close to the known-bag checkpoint
  unless the new run has worse visibility or faster arm motion.
- p95 rotation residual is reasonably close to the known-bag checkpoint.

If the calibration script reports fewer than `100` inliers, keep the bag as a
runtime/topic validation artifact but do not refresh
`config/markers/arm_marker_extrinsics.yaml` from it. A lower-threshold rerun
such as `--min-inliers 50` can be useful for residual characterization, but it
does not meet the first accepted-calibration gate.

## 9. Cleanup And Commit Hygiene

Generated overlay artifacts should stay out of commits:

```text
docker_ws/build_overlay/
docker_ws/install_overlay/
docker_ws/log_overlay/
```

If Docker creates root/nobody-owned generated files in the source tree, remove
only those generated files from inside Docker or fix ownership only for
generated artifacts.

Good source files to commit after successful validation:

```text
multi_cam_localization/sensor_fusion_msgs/msg/DynamicMarkerObservation.msg
multi_cam_localization/sensor_fusion_msgs/CMakeLists.txt
multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py
multi_cam_localization/sensor_fusion_bringup/scripts/calibrate_arm_marker_extrinsic.py
multi_cam_localization/sensor_fusion_bringup/scripts/head_derived_arm_pose_preview_node.py
multi_cam_localization/sensor_fusion_bringup/launch/head_derived_arm_pose_preview.launch.py
multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
multi_cam_localization/sensor_fusion_bringup/config/rviz/phase2_dual_openvins_head_preview.rviz
multi_cam_localization/sensor_fusion_bringup/docs/phase2_dynamic_id2_marker_calibration_validation.md
multi_cam_localization/sensor_fusion_bringup/CMakeLists.txt
multi_cam_localization/sensor_fusion_bringup/test/test_aruco_marker_pose_math.py
```

Do not commit ROS bags or `/tmp` reports unless a separate data-sharing decision
is made.
