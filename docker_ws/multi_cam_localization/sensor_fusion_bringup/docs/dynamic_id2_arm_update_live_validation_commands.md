# Dynamic ID2 Arm Update Live Validation Commands

This runbook starts the dynamic ID2 path conservatively:

1. First run measurement-only: dynamic measurements are produced and consumed by arm OpenVINS, but they do not mutate the arm state.
2. Use RViz and status topics to check frames, timing, covariance, gates, and innovation.
3. Only then restart the arm OpenVINS node with real dynamic updates enabled.

Do not run a build during live validation on the Jetson. Close memory-heavy processes first, especially browsers and old RViz sessions.

## Terminal Setup

Run every command below from a new terminal unless noted otherwise.

```bash
cd /home/robotlab/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
```

If RViz cannot open a display from Docker, run this once on the host:

```bash
xhost +si:localuser:root
```

## Measurement-Only Live Run

This is the recommended first live test. It publishes dynamic arm pose measurements and arm OpenVINS status, but `dynamic_arm_measurement_only:=true` prevents state mutation.

### 1. Head Camera And Head OpenVINS

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py start_camera:=true use_sim_time:=false verbosity:=INFO'
```

### 2. Arm Camera And Arm OpenVINS, Dynamic Measurement-Only

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py start_camera:=true use_sim_time:=false verbosity:=INFO use_dynamic_arm_pose_updates:=true dynamic_arm_measurement_only:=true dynamic_arm_noise_multiplier:=4.0'
```

### 3. Head Marker Node

This publishes fixed ID0 observations and dynamic ID2 observations from the head camera. ID2 stays separate from `marker_fixed_ids`.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py use_sim_time:=false'
```

### 4. Arm Marker Node

This preserves the existing fixed ID0 arm update path.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py use_sim_time:=false'
```

### 5. Dynamic Arm Measurement Producer

This is the new OpenVINS-facing measurement producer. It is separate from the debug preview path.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_arm_pose_measurement.launch.py use_sim_time:=false publish_dynamic_arm_pose_observation:=true require_stable_dynamic_marker:=true'
```

### 6. Optional Debug Preview For RViz

This is visual/debug-only. It should not be used as proof that the OpenVINS update is correct.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup head_derived_arm_pose_preview.launch.py use_sim_time:=false require_stable_dynamic_marker:=true publish_tf:=true'
```

### 7. RViz

Use the preview RViz config because it already contains both OpenVINS paths and the debug head-derived arm overlay.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && rviz2 -d install_overlay/sensor_fusion_bringup/share/sensor_fusion_bringup/config/rviz/phase2_dual_openvins_head_preview.rviz'
```

## Status Checks

Run these in extra terminals while the measurement-only run is active.

Measurement producer status:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /arm/marker_pose/dynamic_arm_measurement/status'
```

Arm OpenVINS dynamic update status:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic echo /ov_msckf_arm/dynamic_arm_update/status'
```

Topic sanity:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /head/marker_pose/dynamic_observation'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 topic hz /arm/marker_pose/dynamic_arm_pose_observation'
```

TF sanity:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo marker_map arm_imu'
```

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 run tf2_ros tf2_echo marker_map head_imu'
```

## What To Look For Before Enabling Real Updates

Good measurement-only behavior:

- `/head/marker_pose/dynamic_observation` is ID2-only.
- `/head/marker_pose/observation` and `/arm/marker_pose/observation` are ID0-only.
- `/arm/marker_pose/dynamic_arm_measurement/status` reports accepted sync modes such as interpolation or nearest, not stale head pose.
- Measurement covariance standard deviations are finite and not tiny.
- `/ov_msckf_arm/dynamic_arm_update/status` reports `measurement_only` or clear skip/reject reasons.
- `skipped_recent_fixed_marker_update` appears when the ID0 skip gate is active.
- RViz keeps separate `marker_map -> head_imu -> head_cam0` and `marker_map -> arm_imu -> arm_cam0` frame chains.
- No sudden arm pose snaps, no non-finite warnings, and no TF frame collisions.

Do not judge success by perfect visual overlap between the preview and arm OpenVINS. The head-derived pose is a noisy measurement, not ground truth.

## Enable Real Dynamic Updates

Only do this after the measurement-only status looks sane. Stop the arm OpenVINS terminal from step 2 and restart it with `dynamic_arm_measurement_only:=false`.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py start_camera:=true use_sim_time:=false verbosity:=INFO use_dynamic_arm_pose_updates:=true dynamic_arm_measurement_only:=false dynamic_arm_noise_multiplier:=4.0 dynamic_arm_skip_after_fixed_marker_s:=0.50 dynamic_arm_max_update_translation_m:=0.35 dynamic_arm_max_update_rotation_deg:=15.0'
```

Keep the measurement producer running with `publish_dynamic_arm_pose_observation:=true`.

Expected real-update behavior:

- Accepted dynamic updates are bounded and logged.
- Large innovations are rejected, not snapped into the state.
- Recent fixed ID0 updates suppress dynamic ID2 updates for the configured skip window.
- Fixed ID0 lock/reanchor behavior remains intact.

## Record A Live Validation Bag

Start this after the system is running and before moving the cameras/markers through the validation motion.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && mkdir -p bags/openvins_tests/phase2_live && ros2 bag record -o bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_$(date +%Y%m%d_%H%M%S) /tf /tf_static /rosout /head/d435i_head/color/image_raw /head/d435i_head/color/camera_info /head/d435i_head/imu /arm/d435i_arm/color/image_raw /arm/d435i_arm/color/camera_info /arm/d435i_arm/imu /head/marker_pose/observation /arm/marker_pose/observation /head/marker_pose/dynamic_observation /arm/marker_pose/dynamic_arm_pose_observation /arm/marker_pose/dynamic_arm_measurement/status /ov_msckf_arm/dynamic_arm_update/status /ov_msckf/poseimu /ov_msckf/odomimu /ov_msckf/pathimu /ov_msckf_arm/poseimu /ov_msckf_arm/odomimu /ov_msckf_arm/pathimu /arm/marker_pose/head_derived/arm_camera_pose /arm/marker_pose/head_derived/path'
```

## Cleanup

Stop each terminal with `Ctrl-C`. Confirm no validation containers are left running:

```bash
docker ps
```

If any old validation container remains, stop it by container id:

```bash
docker stop <container_id>
```
