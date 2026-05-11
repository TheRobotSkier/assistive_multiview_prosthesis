# Phase 2 Dual OpenVINS TF Validation

Use this recipe to validate the per-instance OpenVINS TF frame update with both
D435i cameras live. The goal is to prove that head and arm Phase 2 OpenVINS can
run together, keep topics separate, publish non-colliding TF, and keep marker ID
`0` updates working.

Expected OpenVINS TF trees:

```text
marker_map -> head_imu -> head_cam0
marker_map -> arm_imu  -> arm_cam0
```

Do not run the original non-Phase-2 OpenVINS launch at the same time. Marker ID
`0` is the shared fixed map marker. Keep arm-mounted marker IDs such as `1` or
`2` out of the fixed marker map for this validation.

Latest no-RViz validation checkpoint:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_tf_phase2_20260511_160757
```

This bag was recorded from the full dual-camera stack without RViz. A read-only
`ros2 bag info` check showed 68.0 s of both head/arm raw inputs, both marker
observation streams, both OpenVINS output groups, `/tf`, `/tf_static`, and
`/rosout`. The recording validates the dual-stack recording workflow, but it is
not an arm-mounted marker calibration bag because the head camera did not see
marker ID `2`.

Replay sampling of recorded outputs confirmed:

```text
/ov_msckf/odomimu.child_frame_id: head_imu
/ov_msckf_arm/odomimu.child_frame_id: arm_imu
/ov_msckf/poseimu.header.frame_id: marker_map
/ov_msckf_arm/poseimu.header.frame_id: marker_map
/head/marker_pose/observation.target_frame: head_imu
/arm/marker_pose/observation.target_frame: arm_imu
sampled head/arm marker IDs: 0
sampled TF child frames: head_imu, head_cam0, arm_imu, arm_cam0
generic OpenVINS TF child frames imu/cam0: none seen in replay sample
```

Latest arm-mounted ID2 calibration checkpoint:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816
```

This bag was recorded without RViz after an earlier RViz run hit the live
OpenVINS timing failure. A read-only `ros2 bag info` check showed 53.5 s and
2.7 GiB with both head/arm raw inputs, marker streams, OpenVINS outputs, `/tf`,
`/tf_static`, and `/rosout`. Replay sampling confirmed the expected head/arm
frame IDs and no generic `imu`/`cam0` OpenVINS TF child frames.

Offline ArUco detection on raw images confirmed the required calibration
visibility:

```text
head marker ID 0 frames: 468
head marker ID 2 frames: 1035
arm marker ID 0 frames: 1457
```

The recorded marker observation topics still report marker ID `0`, which is
expected. Marker ID `2` is present in the raw head images and is intended for the
future dynamic-marker observation/calibration path, not the fixed marker-map EKF
update path.

Earlier interrupted/secondary ID2 bag:

```text
docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164627
```

## 0. Before Starting

Use Docker/Jazzy, not host ROS. Close heavy apps before live dual-camera testing
or recording.

Check for old containers:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
docker ps
```

If stale containers are still running, stop only the stale validation containers:

```bash
docker stop <container_id>
```

Confirm the overlay was built after the TF-frame change:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 interface show sensor_fusion_msgs/msg/MarkerPoseObservation && \
   ros2 pkg executables ov_msckf | grep run_subscribe_msckf_marker && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py --show-args'
```

Expected launch arguments:

- Head and arm OpenVINS launches expose `verbosity`, `start_camera`, and
  `use_sim_time`.
- Head and arm marker launches expose `config_file`, `use_sim_time`, and
  `marker_detection_rate_hz`.

If this fails because the overlay is stale, rebuild with the low-memory command
from `phase2_openvins_marker_validation.md`.

## 1. Start Both OpenVINS Instances

OpenVINS and RealSense stay at the calibrated/default `640x480x30` RGB profile.
The marker nodes run their ArUco detector at 15 Hz by default to reduce Python
CPU load while keeping VIO image tracking at 30 Hz.

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

Keep both cameras stationary first. Do the normal acceleration jerk for each
camera so both OpenVINS instances initialize. The two OpenVINS topics must stay
separate:

```text
/ov_msckf/poseimu
/ov_msckf/odomimu
/ov_msckf/pathimu
/ov_msckf_arm/poseimu
/ov_msckf_arm/odomimu
/ov_msckf_arm/pathimu
```

## 2. Start Both Marker Nodes

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

Show marker ID `0` to each camera when you want that camera to receive marker
updates. The marker observations should use:

```text
/head/marker_pose/observation target_frame: head_imu
/arm/marker_pose/observation  target_frame: arm_imu
```

## 3. Open RViz2

Terminal 5:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/phase2_dual_openvins.rviz'
```

RViz fixed frame should be `marker_map`. Watch:

- `marker_map -> head_imu -> head_cam0`
- `marker_map -> arm_imu -> arm_cam0`
- `/ov_msckf/pathimu`
- `/ov_msckf_arm/pathimu`
- `/head/marker_pose/camera_body_pose`
- `/arm/marker_pose/camera_body_pose`

Marker-hidden drift check:

1. Let both OpenVINS instances initialize.
2. Show marker ID `0` to one or both cameras and confirm marker updates.
3. Hide marker ID `0`.
4. Move gently and confirm RViz still shows both OpenVINS poses/paths.
5. Show marker ID `0` again and confirm marker updates continue.

## 4. Run Frame And Topic Checks

Run these after both OpenVINS instances have initialized.

Terminal 6, topic list:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic list | grep -E "/ov_msckf(_arm)?/(poseimu|odomimu|pathimu)|/(head|arm)/marker_pose/observation"'
```

Check OpenVINS output frames:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   echo "head odom child:" && ros2 topic echo --once /ov_msckf/odomimu --field child_frame_id && \
   echo "arm odom child:" && ros2 topic echo --once /ov_msckf_arm/odomimu --field child_frame_id && \
   echo "head pose header:" && ros2 topic echo --once /ov_msckf/poseimu --field header && \
   echo "arm pose header:" && ros2 topic echo --once /ov_msckf_arm/poseimu --field header'
```

Expected:

```text
head odom child: head_imu
arm odom child: arm_imu
pose header frame_id: marker_map
```

Check marker observation target frames after marker ID `0` is visible:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   echo "head marker target:" && ros2 topic echo --once /head/marker_pose/observation --field target_frame && \
   echo "arm marker target:" && ros2 topic echo --once /arm/marker_pose/observation --field target_frame'
```

Expected:

```text
head marker target: head_imu
arm marker target: arm_imu
```

Check TF lookups:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   timeout 5s ros2 run tf2_ros tf2_echo marker_map head_imu && \
   timeout 5s ros2 run tf2_ros tf2_echo head_imu head_cam0 && \
   timeout 5s ros2 run tf2_ros tf2_echo marker_map arm_imu && \
   timeout 5s ros2 run tf2_ros tf2_echo arm_imu arm_cam0'
```

Check for accidental generic OpenVINS frames:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   timeout 5s ros2 topic echo /tf | grep -E "child_frame_id: (imu|cam0)$" || true'
```

Expected: no output from the final grep.

## 5. Record A Dual Validation Bag

For the most useful future replay, start recording while both cameras are still
stationary and before the OpenVINS initialization jerks. Close RViz before long
recordings if CPU is high; the bag contains `/tf`, `/tf_static`, paths, odometry,
and marker overlays for later RViz replay.

Terminal 7:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   mkdir -p bags/openvins_tests/phase2_live && \
   BAG=bags/openvins_tests/phase2_live/dual_openvins_tf_phase2_$(date +%Y%m%d_%H%M%S) && \
   ros2 bag record -o "$BAG" \
     /head/d435i_head/color/image_raw \
     /head/d435i_head/color/camera_info \
     /head/d435i_head/imu \
     /arm/d435i_arm/color/image_raw \
     /arm/d435i_arm/color/camera_info \
     /arm/d435i_arm/imu \
     /head/marker_pose/observation \
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
2. Start the bag recorder while both cameras are stationary.
3. Record a few seconds stationary.
4. Initialize head OpenVINS.
5. Initialize arm OpenVINS.
6. Show marker ID `0` to the head camera.
7. Show marker ID `0` to the arm camera.
8. Move gently with marker ID `0` visible when possible.
9. Hide marker ID `0`; for no-RViz recordings, inspect the drift later from the
   bag, or use RViz only for a short visualization run.
10. Show marker ID `0` again and confirm marker updates resume.
11. Stop the recorder with `Ctrl+C`.

## 6. Quick Bag Info

After recording:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 bag info bags/openvins_tests/phase2_live/<dual_bag_name>'
```

Expected recorded topics include the six raw head/arm inputs, both marker
observation topics, both OpenVINS output groups, `/tf`, `/tf_static`, and
`/rosout`.

Known good no-RViz metadata checkpoint:

```text
Bag: docker_ws/bags/openvins_tests/phase2_live/dual_openvins_tf_phase2_20260511_160757
Duration: 68.0 s
Size: 3.4 GiB
Recorded groups: raw head/arm inputs, marker observations, OpenVINS outputs,
/tf, /tf_static, /rosout
Replay sample: head_imu/arm_imu odom children, marker_map pose headers,
head_imu/arm_imu marker targets, no generic imu/cam0 TF child frames
Limitation: head camera did not see arm-mounted marker ID 2
```

Known good ID2 calibration metadata checkpoint:

```text
Bag: docker_ws/bags/openvins_tests/phase2_live/dual_openvins_id2_calib_phase2_20260511_164816
Duration: 53.5 s
Size: 2.7 GiB
Recorded groups: raw head/arm inputs, marker observations, OpenVINS outputs,
/tf, /tf_static, /rosout
Replay sample: head_imu/arm_imu odom children, marker_map pose headers,
head_imu/arm_imu marker targets, no generic imu/cam0 TF child frames
Raw ArUco visibility: head ID0, head ID2, arm ID0
```

## 7. Future Raw Replay Test From The Dual Bag

Use this later to regenerate marker/OpenVINS outputs from only raw data.

Terminal 1, head OpenVINS without live camera:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py \
     start_camera:=false \
     use_sim_time:=true \
     verbosity:=INFO'
```

Terminal 2, arm OpenVINS without live camera:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py \
     start_camera:=false \
     use_sim_time:=true \
     verbosity:=INFO'
```

Terminal 3, head marker node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py use_sim_time:=true marker_detection_rate_hz:=15.0'
```

Terminal 4, arm marker node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py use_sim_time:=true marker_detection_rate_hz:=15.0'
```

Terminal 5, replay only raw inputs:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 bag play bags/openvins_tests/phase2_live/<dual_bag_name> --clock --topics \
     /head/d435i_head/color/image_raw \
     /head/d435i_head/color/camera_info \
     /head/d435i_head/imu \
     /arm/d435i_arm/color/image_raw \
     /arm/d435i_arm/color/camera_info \
     /arm/d435i_arm/imu'
```

Do not replay recorded marker observations, OpenVINS outputs, `/tf`, or
`/tf_static` into a fresh validation run.

## 8. Acceptance Criteria

The live validation passes when:

- `/ov_msckf/*` and `/ov_msckf_arm/*` remain separate.
- Head and arm OpenVINS both initialize.
- All OpenVINS pose/path headers use `marker_map`.
- `/ov_msckf/odomimu.child_frame_id` is `head_imu`.
- `/ov_msckf_arm/odomimu.child_frame_id` is `arm_imu`.
- `/head/marker_pose/observation.target_frame` is `head_imu`.
- `/arm/marker_pose/observation.target_frame` is `arm_imu`.
- Phase 2 marker nodes run with `marker_detection_rate_hz:=15.0`.
- TF contains `marker_map -> head_imu -> head_cam0`.
- TF contains `marker_map -> arm_imu -> arm_cam0`.
- No OpenVINS TF collision occurs on generic `imu` or `cam0`.
- RViz continues showing both OpenVINS paths while marker ID `0` is hidden.
- Marker ID `0` updates resume when the marker returns.
