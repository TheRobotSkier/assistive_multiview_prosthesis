# Head D435i + OpenVINS + Marker Pose Bag Replay Validation

Run all ROS 2 commands inside the Jazzy `realsense_camera` Docker container.
The host may have ROS 2 Humble, so do not use host ROS commands for this
validation.

RViz2 config:

```text
/miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/head_openvins.rviz
```

Recommended bag directory in the container:

```text
/miahand_ws/src/bags/openvins_tests/head_marker_validation
```

## Why Record TF And More Topics?

For a quick marker-node replay, the minimum input topics are:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/imu
/ov_msckf/odomimu
```

For Phase 2 comparison and debugging, record more:

```text
/tf
/tf_static
/rosout
/head/d435i_head/color/image_raw
/head/d435i_head/color/camera_info
/head/d435i_head/imu
/ov_msckf/odomimu
/ov_msckf/poseimu
/ov_msckf/pathimu
/head/marker_pose/*
```

`/tf` and `/tf_static` are useful because RViz and later debug scripts can
recreate the transform tree from the live run. This makes the bag a better
baseline when comparing Phase 1 external marker correction with future Phase 2
OpenVINS-internal marker correction.

## Completed Validation Result

Validation performed:

- Terminal 1: started D435i + OpenVINS.
- Terminal 2: started marker pose node.
- Terminal 3: opened RViz2 for live recording with `head_openvins.rviz`.
- Ran manual reanchor test, also while recording.
- Recorded compact validation bag.
- Recorded full validation bag.
- Terminal A1: played recorded-output bag.
- Terminal A2: opened RViz2 for recorded-output replay.
- Terminal A3: inspected replayed marker topics.
- Terminal B1: started raw replay container shell.
- Terminal B2: started marker node during raw replay.
- Terminal B3: opened RViz2 for raw-input replay.
- Terminal B4: inspected regenerated marker outputs.

Recorded motion sequence:

1. OpenVINS initialized successfully.
2. Marker was visible while VIO was good.
3. Camera moved away from marker while VIO was still good.
4. Camera returned to marker while VIO was still good.
5. This may have repeated a few times.
6. Camera moved away to a texture-only view where VIO began drifting.
7. Camera turned back to marker while VIO was bad.
8. Manual reanchor was sent.
9. Recording was stopped.

Observed behavior:

- During good-VIO segments, marker-corrected odom behaved as expected.
- During the bad-VIO/drifting segment, corrected odom snapped/reanchored back
  toward the correct marker-map pose when marker was visible.
- Between marker corrections, output drifted because raw OpenVINS was still
  drifting.
- Pose covariance continued growing in the bad-VIO case.
- This is expected for Phase 1 external correction because it does not update
  the internal OpenVINS EKF state, velocity, or covariance.
- This motivates Phase 2: marker update/reanchor inside OpenVINS EKF, including
  pose, velocity, and covariance.

## Actual Validation Bags Recorded On 2026-05-06

### `head_marker_phase1_full_validation_20260506_124409`

```text
Bag size: 1.8 GiB
Duration: 71.218826315 s
Start: May  6 2026 12:44:10.386254153
End:   May  6 2026 12:45:21.605080468
Messages: 88992
```

Important counts:

```text
/head/d435i_head/color/image_raw: 1903
/head/d435i_head/color/camera_info: 2134
/head/d435i_head/imu: 14073
/ov_msckf/odomimu: 8609
/ov_msckf/poseimu: 1742
/ov_msckf/pathimu: 1742
/head/marker_pose/imu_pose: 654
/head/marker_pose/ov_corrected_odom: 8623
/head/marker_pose/reanchor_event: 105
/tf: 29418
/tf_static: 1
```

### `head_marker_phase1_full_validation_20260506_122604`

```text
Bag size: 2.0 GiB
Duration: 79.842433792 s
Start: May  6 2026 12:26:05.301420442
End:   May  6 2026 12:27:25.143854234
Messages: 91200
```

Important counts:

```text
/head/d435i_head/color/image_raw: 2130
/head/d435i_head/color/camera_info: 2393
/head/d435i_head/imu: 15768
/ov_msckf/odomimu: 8016
/ov_msckf/poseimu: 1973
/ov_msckf/pathimu: 1973
/head/marker_pose/imu_pose: 868
/head/marker_pose/ov_corrected_odom: 7937
/head/marker_pose/reanchor_event: 171
/tf: 28695
/tf_static: 1
```

## Commands

### Terminal 1: Start D435i + OpenVINS

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose stop realsense_camera 2>/dev/null || true
docker rm -f openvins_phase1_live 2>/dev/null || true

docker compose run --rm --name openvins_phase1_live --service-ports --entrypoint /bin/bash realsense_camera -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 launch sensor_fusion_bringup head_d435i_openvins.launch.py \
    rviz_enable:=false \
    verbosity:=INFO
'
```

### Terminal 2: Start Marker Pose Node

```bash
docker exec -it openvins_phase1_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 launch sensor_fusion_bringup head_marker_pose.launch.py
'
```

### Terminal 3: Open RViz2

```bash
docker exec -it openvins_phase1_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/head_openvins.rviz
'
```

### Manual Reanchor

```bash
docker exec -it openvins_phase1_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 service call /head/marker_pose/request_reanchor std_srvs/srv/Trigger {}
'
```

### Record Compact Validation Bag

The compact bag keeps enough data to inspect the marker-corrected output and
basic replay behavior.

```bash
docker exec -it openvins_phase1_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  mkdir -p /miahand_ws/src/bags/openvins_tests/head_marker_validation
  cd /miahand_ws/src/bags/openvins_tests/head_marker_validation

  BAG=head_marker_phase1_compact_validation_$(date +%Y%m%d_%H%M%S)
  ros2 bag record -o "$BAG" \
    /head/d435i_head/color/image_raw \
    /head/d435i_head/imu \
    /ov_msckf/odomimu \
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
    /tf_static
'
```

### Record Full Validation Bag

The full bag includes raw camera metadata, OpenVINS pose/path topics, marker
outputs, TF, and logs. This is the preferred bag for future Phase 2 comparison.

```bash
docker exec -it openvins_phase1_live bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  mkdir -p /miahand_ws/src/bags/openvins_tests/head_marker_validation
  cd /miahand_ws/src/bags/openvins_tests/head_marker_validation

  BAG=head_marker_phase1_full_validation_$(date +%Y%m%d_%H%M%S)
  ros2 bag record -o "$BAG" \
    /head/d435i_head/color/image_raw \
    /head/d435i_head/color/camera_info \
    /head/d435i_head/imu \
    /ov_msckf/odomimu \
    /ov_msckf/poseimu \
    /ov_msckf/pathimu \
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

### Find Latest Validation Bags

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm --entrypoint /bin/bash realsense_camera -lc '
  source /opt/ros/jazzy/setup.bash
  export TZ=Europe/Copenhagen

  cd /miahand_ws/src/bags/openvins_tests/head_marker_validation

  echo "=== newest validation bags ==="
  ls -1td head_marker_phase1_*_validation_* | head -n 10

  echo
  echo "=== info for newest two bags ==="
  for BAG in $(ls -1td head_marker_phase1_*_validation_* | head -n 2); do
    echo
    echo "========================================"
    echo "BAG: $BAG"
    echo "========================================"
    ros2 bag info "$BAG"
  done
'
```

### Terminal A1: Replay Recorded Outputs

This replay uses the recorded marker outputs. It is useful for inspecting the
captured Phase 1 result in RViz exactly as recorded.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker rm -f openvins_phase1_output_replay 2>/dev/null || true

docker compose run --rm --name openvins_phase1_output_replay --service-ports --entrypoint /bin/bash realsense_camera -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  cd /miahand_ws/src/bags/openvins_tests/head_marker_validation
  ros2 bag play head_marker_phase1_full_validation_20260506_124409 --clock
'
```

### Terminal A2: Open RViz2 During Recorded-Output Replay

```bash
docker exec -it openvins_phase1_output_replay bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/head_openvins.rviz
'
```

### Terminal A3: Inspect Replayed Marker Topics

```bash
docker exec -it openvins_phase1_output_replay bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 topic echo --once /head/marker_pose/marker_valid
  ros2 topic echo --once /head/marker_pose/marker_quality
  ros2 topic echo --once /head/marker_pose/reanchor_event
  ros2 topic hz /head/marker_pose/ov_corrected_odom
'
```

### Terminal B1: Replay Raw Inputs Only

This replay excludes recorded `/head/marker_pose/*` topics so the marker node
can regenerate outputs from the raw image stream and recorded OpenVINS odom.
These May 2026 bags used a marker ID 0 that was later measured at 153.3 mm, so
the marker node should use `head_aruco_map_replay_1533mm.yaml` for regenerated
outputs. The default `head_aruco_map.yaml` is now reserved for the final
100 mm marker setup.

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker rm -f openvins_phase1_raw_replay 2>/dev/null || true

docker compose run --rm --name openvins_phase1_raw_replay --service-ports --entrypoint /bin/bash realsense_camera -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  cd /miahand_ws/src/bags/openvins_tests/head_marker_validation
  ros2 bag play head_marker_phase1_full_validation_20260506_124409 --clock \
    --topics \
    /head/d435i_head/color/image_raw \
    /head/d435i_head/color/camera_info \
    /head/d435i_head/imu \
    /ov_msckf/odomimu \
    /tf_static
'
```

### Terminal B2: Start Marker Node During Raw Replay

```bash
docker exec -it openvins_phase1_raw_replay bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 launch sensor_fusion_bringup head_marker_pose.launch.py \
    config_file:=/miahand_ws/src/install_overlay/sensor_fusion_bringup/share/sensor_fusion_bringup/config/markers/head_aruco_map_replay_1533mm.yaml
'
```

### Terminal B3: Open RViz2 During Raw-Input Replay

```bash
docker exec -it openvins_phase1_raw_replay bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  rviz2 -d /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/config/rviz/head_openvins.rviz
'
```

### Terminal B4: Inspect Regenerated Marker Outputs

```bash
docker exec -it openvins_phase1_raw_replay bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  ros2 topic echo --once /head/marker_pose/marker_valid
  ros2 topic echo --once /head/marker_pose/active_marker_id
  ros2 topic echo --once /head/marker_pose/marker_quality
  ros2 topic echo --once /head/marker_pose/reanchor_event
  ros2 topic hz /head/marker_pose/ov_corrected_odom
'
```

## Pass/Fail Checklist

```text
[x] Live D435i + OpenVINS launched.
[x] OpenVINS initialized and /ov_msckf/odomimu published.
[x] Marker node launched.
[x] Marker 0 detected live.
[x] Manual reanchor returned success=True.
[x] RViz2 opened with head_openvins.rviz for live recording.
[x] Live marker_map axis check passed.
[x] Compact validation bag recorded.
[x] Full validation bag recorded.
[x] Bag info showed expected raw image, camera_info, IMU, OpenVINS odom, marker topics, /tf, and /tf_static.
[x] Replay option A opened in RViz2 and showed recorded marker outputs.
[x] Replay option B regenerated marker outputs from raw image + odom.
[x] reanchor_event JSON contained accepted/rejected reasons.
[x] Bad-VIO segment reanchored when marker became visible again.
[x] Continued bad-VIO drift and covariance growth were interpreted as expected Phase 1 external-correction behavior.
[ ] No generated build_overlay/install_overlay/log_overlay artifacts are committed.
```
