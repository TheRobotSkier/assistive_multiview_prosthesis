# Phase 2 Dual OpenVINS TF Validation

Use this recipe after building the per-instance OpenVINS TF frame update. The
goal is to run the head and arm Phase 2 stacks together without TF collisions.

Expected OpenVINS TF trees:

```text
marker_map -> head_imu -> head_cam0
marker_map -> arm_imu  -> arm_cam0
```

Marker ID `0` remains the shared fixed map marker. Keep marker ID `1` out of
the fixed marker map for this Phase 2 validation.

## 1. Smoke Checks

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py --show-args'
```

Confirm the head and arm OpenVINS launches expose `start_camera` and
`use_sim_time`.

## 2. Start Both OpenVINS Instances

Terminal 1, head:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py verbosity:=INFO'
```

Terminal 2, arm:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_d435i_openvins_phase2.launch.py verbosity:=INFO'
```

Keep both cameras stationary first, then initialize each OpenVINS instance with
the normal acceleration jerk.

## 3. Start Marker Observations

Terminal 3, head marker node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py'
```

Terminal 4, arm marker node:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 launch sensor_fusion_bringup arm_marker_pose_phase2.launch.py'
```

## 4. Frame And Topic Checks

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic echo --once /ov_msckf/odomimu --field child_frame_id && \
   ros2 topic echo --once /ov_msckf_arm/odomimu --field child_frame_id && \
   ros2 topic echo --once /head/marker_pose/observation --field target_frame && \
   ros2 topic echo --once /arm/marker_pose/observation --field target_frame'
```

Expected values:

```text
head_imu
arm_imu
head_imu
arm_imu
```

Check TF frames:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 topic echo --once /tf'
```

The `/tf` stream should contain transforms for `head_imu`, `head_cam0`,
`arm_imu`, and `arm_cam0`. It should not contain OpenVINS transforms with
generic child frames `imu` or `cam0`.

## 5. RViz Drift Check

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/phase2_dual_openvins.rviz'
```

Use `marker_map` as the fixed frame. With marker ID `0` hidden, RViz should
continue showing both OpenVINS poses and paths from TF/odometry. When marker ID
`0` returns, both marker update paths should continue accepting marker ID `0`
updates and the marker-derived poses should line up with the OpenVINS poses.

## 6. Record A Short Dual Validation Bag

Start recording before marker reacquisition or any intentional drift test:

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
     /arm/marker_pose/observation \
     /head/marker_pose/camera_body_pose \
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

Acceptance criteria:

- `/ov_msckf/*` and `/ov_msckf_arm/*` remain separate.
- All OpenVINS pose/path headers use `marker_map`.
- Head odometry and marker observations use `head_imu`.
- Arm odometry and marker observations use `arm_imu`.
- TF contains `head_imu`, `head_cam0`, `arm_imu`, and `arm_cam0`.
- No OpenVINS TF collision occurs on generic `imu` or `cam0`.
