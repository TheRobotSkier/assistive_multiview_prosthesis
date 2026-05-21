# Arm Marker ID2 Extrinsic Calibration Commands

Use this when a new physical mount changes the fixed transform between marker
ID `2` and the arm D435i camera. The calibration refreshes:

```text
multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
```

The dynamic ID2 OpenVINS update path depends on this file, but the calibration
itself should be recorded in a safe observation mode. Do not judge dynamic ID2
update/reanchor quality until this transform has been refreshed for the final
fixture.

## Latest Accepted Calibration

The replacement arm-mounted marker ID2 fixture was calibrated on 2026-05-21 and
installed into:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
```

Source bag:

```text
bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260521_144725
```

Temporary candidate/report directory:

```text
docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate_20260521_144725/
```

Calibration summary:

```text
Bag duration: 120.006 s
Raw image counts: head=2760, arm=3180
Detection counts: head ID0=2649, head ID2=1025, head ID0+ID2=997, arm ID0=1790
Synchronized calibration samples: 573
Inliers/outliers: 561 / 12
Median translation residual: 0.0060 m
P95 translation residual: 0.0118 m
Median rotation residual: 1.429 deg
P95 rotation residual: 3.888 deg
```

The generated frames are expected:

```text
marker_frame: arm_marker_2
parent_camera_frame: arm_d435i_arm_color_optical_frame
parent_imu_frame: arm_imu
```

This supersedes the accepted 2026-05-19 calibration from
`dynamic_id2_arm_update_live_20260519_113936`, which had `166` inliers,
p95 translation residual `0.0104 m`, and p95 rotation residual `2.349 deg`.
The 2026-05-21 bag has many more synchronized samples and still keeps residuals
in the documented acceptance range.

No CAD screw-frame sanity check is recorded here for the 2026-05-21 replacement
mount. If a future CAD comparison is needed, remember that the D435i bottom
screw/tripod frame is not the same as the ROS optical frame, so translation
values cannot be compared directly without applying the correct frame rotation.

ID2 remains dynamic only. Do not add ID2 to `marker_fixed_ids`.

## What The Calibration Needs

The offline calibration script detects markers from raw images in the bag. It
does not use dynamic ID2 as a fixed marker-map landmark.

For a good calibration bag, record motion where:

- the head D435i sees fixed marker ID `0` and arm-mounted marker ID `2` at the
  same time
- the arm D435i sees fixed marker ID `0`
- the three views overlap in time for many poses
- marker ID `2` is the correct printed size from
  `head_aruco_map.yaml` (`dynamic_markers.2.size_m`, currently `0.100`)
- marker ID `2` is not occluded by the prosthesis through the normal motion
  range

Target at least `100` calibration inliers before accepting a new checked-in
transform.

## 1. Start From The Docker Workspace

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
docker ps
```

If a stale validation container is still running, stop only that stale
container:

```bash
docker stop <container_id>
```

## 2. Build The Required Packages

Run this after code changes or after a clean restart:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && colcon --log-base log_overlay build --symlink-install --build-base build_overlay --install-base install_overlay --executor sequential --parallel-workers 1 --packages-select sensor_fusion_msgs sensor_fusion_bringup ov_msckf'
```

Smoke-check that the calibration executable is installed:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 pkg executables sensor_fusion_bringup | grep calibrate_arm_marker_extrinsic'
```

## 3. Check Marker Size Before Recording

The configured ID2 marker size must match the physical printed marker:

```bash
cd ~/Documents/assistive_multiview_prosthesis
grep -nA4 "dynamic_markers:" docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml
```

If the redesigned fixture uses a different printed marker size, update
`dynamic_markers.2.size_m` before recording the accepted calibration bag.

## 4. Record A Calibration Bag

Use observation mode for calibration. This records raw images and marker topics
without letting an old ID2 extrinsic actively reanchor the arm estimator.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py mode:=observe start_rviz:=false record_bag:=true'
```

Recommended capture sequence:

1. Start with both cameras stationary.
2. Let both OpenVINS instances initialize.
3. Show fixed marker ID `0` to the head camera.
4. Show fixed marker ID `0` to the arm camera.
5. Bring arm-mounted marker ID `2` into the head camera view.
6. Move the arm gently through many poses while keeping ID2 visible to the head
   camera and ID0 visible to the arm camera as often as possible.
7. Keep the marker away from image borders and avoid prosthesis occlusion.
8. Stop with `Ctrl+C` so the MCAP closes cleanly.

The launch records to:

```text
bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_<timestamp>
```

## 5. Inspect The New Bag

Replace `<bag_name>` with the directory printed by the recorder.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 bag info bags/openvins_tests/phase2_live/<bag_name>'
```

Minimum topics for calibration:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/color/camera_info
/arm/d435i_arm/color/image_raw
/arm/d435i_arm/color/camera_info
```

Useful extra topics for debugging:

```text
/head/marker_pose/dynamic_observation
/head/marker_pose/observation
/arm/marker_pose/observation
/ov_msckf/odomimu
/ov_msckf_arm/odomimu
/tf
/tf_static
```

## 6. Run Calibration To A Temporary File

First write the result to a local candidate directory so the checked-in
calibration is not replaced until the residuals look good.

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && rm -rf calibration/arm/d435i_310622071850/arm_marker_id2_candidate && ros2 run sensor_fusion_bringup calibrate_arm_marker_extrinsic.py --bag bags/openvins_tests/phase2_live/<bag_name> --head-config multi_cam_localization/sensor_fusion_bringup/config/markers/head_aruco_map.yaml --arm-config multi_cam_localization/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml --dynamic-marker-id 2 --fixed-marker-id 0 --output calibration/arm/d435i_310622071850/arm_marker_id2_candidate/arm_marker_extrinsics.yaml --report-dir calibration/arm/d435i_310622071850/arm_marker_id2_candidate/report --sync-tolerance-s 0.05 --min-inliers 100'
```

Print the candidate and the residual report:

```bash
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && sed -n "1,220p" calibration/arm/d435i_310622071850/arm_marker_id2_candidate/arm_marker_extrinsics.yaml && sed -n "1,220p" calibration/arm/d435i_310622071850/arm_marker_id2_candidate/report/arm_marker_extrinsic_summary.yaml && head -20 calibration/arm/d435i_310622071850/arm_marker_id2_candidate/report/arm_marker_extrinsic_residuals.csv'
```

## 7. Acceptance Checks

Accept the new transform only if:

- the script reports at least `100` inliers
- p95 translation residual is roughly in the centimeter range
- p95 rotation residual is only a few degrees
- the generated `marker_frame` is `arm_marker_2`
- the generated `parent_camera_frame` is `arm_d435i_arm_color_optical_frame`
- the generated `parent_imu_frame` is `arm_imu`
- the bag had good ID2 visibility without repeated prosthesis occlusion
- ID2 still does not appear in any `marker_fixed_ids`

For reference, the old accepted calibration-source bag produced `173` inliers,
p95 translation residual `0.0273 m`, and p95 rotation residual `3.541 deg`.

If the bag has fewer than `100` inliers, keep it as a diagnostic recording but
do not update the checked-in extrinsic from it.

## 8. Install The Accepted Transform

Only after the temporary result passes the acceptance checks, copy it into the
checked-in config:

```bash
cp docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate/arm_marker_extrinsics.yaml docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
```

Review the diff:

```bash
git diff -- docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml
```

## 9. Validate Dynamic ID2 With The New Transform

After installing the new extrinsic, run the default active workflow and record a
short validation bag:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment
docker compose run --rm realsense_camera 'source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && ros2 launch sensor_fusion_bringup dynamic_id2_arm_update_live.launch.py start_rviz:=true record_bag:=true'
```

This post-calibration bag should contain overlapping messages on:

```text
/ov_msckf_arm/odomimu
/arm/marker_pose/dynamic_arm_pose_observation
/ov_msckf_arm/dynamic_arm_update/status
```

If `/ov_msckf_arm/odomimu` stops before dynamic measurements begin, debug the arm
OpenVINS runtime first. That bag cannot validate dynamic ID2 update/reanchor
behavior.

## 10. Commit

After a successful calibration and active validation:

```bash
git add docker_ws/multi_cam_localization/sensor_fusion_bringup/config/markers/arm_marker_extrinsics.yaml docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/arm_marker_id2_extrinsic_calibration_commands.md
git commit -m "Update arm marker ID2 extrinsic calibration"
```

Do not commit ROS bags, `/tmp` reports, `build_overlay`, `install_overlay`, or
`log_overlay`. Do not commit the candidate directory unless you intentionally
want to archive the calibration report:

```text
docker_ws/calibration/arm/d435i_310622071850/arm_marker_id2_candidate/
```
