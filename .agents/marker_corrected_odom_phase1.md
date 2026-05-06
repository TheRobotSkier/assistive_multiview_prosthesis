# Phase 1 Marker-Corrected Odom Handoff

## Scope

Phase 1 implements an external ArUco marker-corrected odom layer for the head
D435i OpenVINS pose. It does not modify OpenVINS internals.

## Implemented

- External marker node publishes raw optical-frame marker pose, marker-derived
  calibrated IMU pose, visualization-only body pose, and corrected OpenVINS
  odom.
- Marker correction maintains an external `T_map_global` transform from
  OpenVINS `global` into `marker_map`.
- Marker detections are timestamp-matched against an OpenVINS odom buffer.
- Initial lock and manual reanchor are supported through
  `/head/marker_pose/request_reanchor`.
- Marker validity and correction acceptance are separate in diagnostics.
- Reanchor events are compact JSON on `/head/marker_pose/reanchor_event`.
- `camera_pose_raw` remains a ROS/OpenCV optical frame.
- `camera_body_pose` was added for RViz only, with `+X` forward, `+Y` left,
  `+Z` up.
- `imu_pose` remains the true calibrated IMU pose in `marker_map`.
- Marker ID 0 map axes were adjusted in YAML so `marker_map` behaves like a
  z-up/navigation frame in live validation.

## Files Changed

- `multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
- `multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml`
- `multi_cam_localization/sensor_fusion_bringup/config/rviz/head_openvins.rviz`
- `multi_cam_localization/sensor_fusion_bringup/docs/marker_correction_validation.md`
- `multi_cam_localization/sensor_fusion_bringup/test/test_aruco_marker_pose_math.py`
- `multi_cam_localization/sensor_fusion_bringup/CMakeLists.txt`
- `multi_cam_localization/sensor_fusion_bringup/package.xml`

## Live Validation Passed

- Jazzy overlay build succeeded inside the `realsense_camera` Docker container.
- `ros2 pkg prefix sensor_fusion_bringup` resolved to
  `/miahand_ws/src/install_overlay/sensor_fusion_bringup`.
- Marker topics publish:
  `/head/marker_pose/active_marker_id`,
  `/head/marker_pose/camera_pose_raw`,
  `/head/marker_pose/camera_body_pose`,
  `/head/marker_pose/imu_pose`,
  `/head/marker_pose/marker_valid`,
  `/head/marker_pose/ov_corrected_odom`,
  `/head/marker_pose/reanchor_event`,
  `/head/marker_pose/vio_valid`.
- Marker ID 0 detection works live.
- Manual reanchor service returned `success=True`.
- `/head/marker_pose/imu_pose` and `/head/marker_pose/ov_corrected_odom` line
  up with each other.
- In RViz with Fixed Frame `marker_map`, marker-map behavior now looks similar
  to the OpenVINS `global` frame.
- Numeric `/head/marker_pose/imu_pose` validation passed: physical upward
  motion increases `marker_map` z.

## Known Caveats

- Do not judge camera viewing direction from the red RViz pose arrow on
  `camera_pose_raw`; raw optical-frame viewing direction is blue `+Z`.
- `camera_body_pose` is display-only and must not be used as a measurement or
  correction frame.
- `imu_pose` axes are the calibrated IMU axes and may not be visually intuitive.
- Covariance, VIO health thresholds, marker gates, periodic correction, and
  reanchor policy should not be tuned further until the frame convention has
  been fully validated.
- The external correction layer does not reset or update the OpenVINS internal
  EKF state.

## Docker/Jazzy Validation Command

Run from:
`~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment`

```bash
docker compose run --rm --entrypoint /bin/bash realsense_camera -lc '
  set -exo pipefail
  source /opt/ros/jazzy/setup.bash
  source /miahand_ws/install/setup.bash
  source /miahand_ws/src/install_overlay/setup.bash

  cd /miahand_ws

  python3 -m py_compile \
    /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py

  python3 -m pytest -q \
    /miahand_ws/src/multi_cam_localization/sensor_fusion_bringup/test/test_aruco_marker_pose_math.py

  MAKEFLAGS="-j1" CMAKE_BUILD_PARALLEL_LEVEL=1 \
    colcon build \
      --executor sequential \
      --symlink-install \
      --packages-select sensor_fusion_bringup \
      --build-base /miahand_ws/src/build_overlay \
      --install-base /miahand_ws/src/install_overlay

  source /miahand_ws/src/install_overlay/setup.bash
  ros2 pkg prefix sensor_fusion_bringup
  ros2 launch sensor_fusion_bringup head_marker_pose.launch.py --show-args
'
```

If `set -u` is added, Jazzy setup may fail on an unbound
`AMENT_TRACE_SETUP_FILES`; use `set -eo pipefail` or `set -exo pipefail`.

## Remaining Validation Before Commit

- Confirm source and installed overlay YAML match after the final Docker build.
- Repeat live RViz check with Fixed Frame `marker_map`.
- Confirm physical upward motion increases numeric `imu_pose.pose.position.z`.
- Confirm closer/farther motion changes a depth/horizontal axis rather than
  `marker_map` z.
- Confirm yaw appears as yaw around `marker_map` z.
- Replay a recorded bag to verify marker visible/lost transitions and
  correction diagnostics.

Do not include generated overlay artifacts in commits:
`build_overlay/`, `install_overlay/`, `log_overlay/`.

## Future Work

- Phase 2: investigate internal OpenVINS EKF marker update/reanchor so marker
  observations can directly correct OpenVINS pose, velocity, and covariance.
- Separate task: create the arm D435i setup from
  `calibration/arm/d435i_310622071850/`.
