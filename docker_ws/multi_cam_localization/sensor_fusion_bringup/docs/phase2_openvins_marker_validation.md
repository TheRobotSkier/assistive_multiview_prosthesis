# Phase 2 OpenVINS Marker Validation

This recipe is for validating the Phase 2 marker-enabled OpenVINS path on the
Jetson. OpenVINS builds can exhaust memory on the Jetson if colcon/CMake builds
in parallel, so keep the OpenVINS build sequential.

For bench commands to try the Phase 2 path live and record a pre-init validation
bag, see `phase2_openvins_live_trial_and_recording.md`.
For simultaneous head and arm TF validation, see
`phase2_dual_openvins_tf_validation.md`.

## Low-Memory Build

Close VS Code, RViz, browsers, and other large applications before building.

Clean only generated overlay artifacts:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws
sudo chown -R "$(id -u):$(id -g)" build_overlay install_overlay log_overlay 2>/dev/null || true
rm -rf build_overlay install_overlay log_overlay
```

Build in the Jazzy Docker environment:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm \
  -e MAKEFLAGS=-j1 \
  -e CMAKE_BUILD_PARALLEL_LEVEL=1 \
  realsense_camera \
  "source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && colcon --log-base log_overlay build \
    --executor sequential --parallel-workers 1 \
    --symlink-install \
    --build-base build_overlay \
    --install-base install_overlay \
    --packages-up-to sensor_fusion_bringup ov_msckf"
```

Known successful checkpoint:

```text
Summary: 6 packages finished [43min 20s]
  3 packages had stderr output: ov_core ov_init ov_msckf
```

The stderr note is expected when the packages emit CMake/Eigen warnings. Treat
it as a failure only if colcon reports a failed package.

## Smoke Checks

After a successful build, verify the new message, executable, and launch files:

```bash
cd ~/Documents/assistive_multiview_prosthesis/docker_ws/docker-deployment

docker compose run --rm realsense_camera \
  "source /opt/ros/jazzy/setup.bash && cd /miahand_ws/src && source install_overlay/setup.bash && \
   ros2 interface show sensor_fusion_msgs/msg/MarkerPoseObservation && \
   ros2 pkg executables ov_msckf | grep run_subscribe_msckf_marker && \
   ros2 launch sensor_fusion_bringup head_d435i_openvins_phase2.launch.py --show-args && \
   ros2 launch sensor_fusion_bringup head_marker_pose_phase2.launch.py --show-args"
```

Expected:

- The head OpenVINS launch exposes `verbosity`, `start_camera`, and
  `use_sim_time`.
- The head marker launch exposes `config_file` and `use_sim_time`.

## Behavior Validation

Run Phase 1 and Phase 2 as separate sessions. Do not run the original OpenVINS
launch and the Phase 2 OpenVINS launch at the same time with the same namespace.

Expected checks:

- Original Phase 1 launches still start unchanged.
- `/head/marker_pose/observation` is published by the Python marker node.
- Phase 2 OpenVINS publishes pose/odom/path in `marker_map`.
- `/ov_msckf/odomimu.child_frame_id` is `head_imu`.
- `/head/marker_pose/observation.target_frame` is `head_imu`.
- OpenVINS TF publishes `marker_map -> head_imu -> head_cam0`.
- Any custom Phase 2 marker config keeps `frames.imu_frame` equal to the
  OpenVINS launch `marker_target_frame`.
- Marker ID 0 is accepted as the fixed map marker.
- Marker ID 1 is not accepted as a fixed map landmark.
- Healthy VIO receives bounded marker EKF corrections.
- Large drift/reacquire uses the explicit marker reset path only when a reliable
  marker-map velocity fit is available.
- The external `/head/marker_pose/ov_corrected_odom` wrapper is not needed in
  Phase 2 validation runs.

## 2026-05-08 100 mm Replay Results

These replays used only `head_aruco_map.yaml`, where marker ID `0` has
`size_m: 0.100`. The older `head_marker_phase1*_20260506_*` bags and the
`1533mm`/`160mm` marker configs were excluded. Replays used only raw input
topics: `/head/d435i_head/color/image_raw`,
`/head/d435i_head/color/camera_info`, and `/head/d435i_head/imu`.

Smoke checks passed for the marker observation message, the
`run_subscribe_msckf_marker` executable, and both Phase 2 launch files. Original
launch `--show-args` checks also passed. The Phase 2 marker launch resolved
`head_aruco_map.yaml` and external correction was disabled.

Ordered bag results:

- `head_marker_cov_validation_100mm_final_stationary_20260507_132957`:
  marker ID 0 observations published (`1516`, with `1515` stable), but OpenVINS
  did not initialize from the raw replay, so no `poseimu`, `odomimu`, `pathimu`,
  reset, or EKF update was produced.
- `head_marker_cov_validation_100mm_final_gentle_motion_20260507_135934`:
  marker ID 0 observations published (`759`, with `756` stable), but OpenVINS
  did not initialize. Logs repeatedly reported static-init rejection because the
  platform was moving too much.
- `head_marker_cov_validation_100mm_final_lost_regained_vio_good_20260507_140742`:
  OpenVINS initialized and published `poseimu`, `odomimu`, and `pathimu` in
  `marker_map`. Marker ID 0 produced one first-lock reset and bounded EKF
  updates; reset requests before the first lock were held until the marker-map
  velocity fit became reliable. No marker stale/timestamp drops were observed.
- `head_marker_cov_validation_100mm_final_lost_regained_vio_drift_20260507_141607`:
  marker ID 0 observations published (`681`, all stable), but OpenVINS did not
  initialize. Logs repeatedly reported static-init rejection because the
  platform was moving too much.

Marker ID 1 rejection was validated with an image-synchronized synthetic ID 1
publisher on the VIO-good replay. OpenVINS logged seven non-fixed-marker
rejections and zero marker-1 accepts or resets.

The successful VIO-good replay did not show the stale/timestamp-drop pattern
that would require changing `ROS2Visualizer` marker queue handling. The other
three 100 mm bags validate marker publication/configuration, but they do not
exercise the internal OpenVINS marker EKF path until the replay can produce a VIO
initialization.

## 2026-05-10 Pre-Init Live Bag Replay Result

The first confirmed full replay-validation bag is:

```text
docker_ws/bags/openvins_tests/phase2_live/head_marker_phase2_100mm_preinit_20260510_122723
```

This bag was recorded with the final 100 mm marker config and starts before the
OpenVINS initialization jerk. It includes multiple larger VIO drift/reacquire
events from live testing where marker ID `0` recovered the correct pose.

Fresh replay used only raw input topics:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/color/camera_info
/head/d435i_head/imu
```

Replay passed the Phase 2 marker EKF behavior checks:

- OpenVINS initialized once from the raw replay.
- Marker node used `head_aruco_map.yaml`.
- `/head/marker_pose/observation` published `879` marker ID `0` observations.
- OpenVINS published `poseimu`, `odomimu`, and `pathimu` in `marker_map`.
- Marker ID `0` produced `4` marker-map resets and `1212` accepted EKF updates.
- `19` reset requests were held until the marker-map velocity fit was reliable.
- No stale marker/timestamp drops were logged.
- The live Propagator assertion did not reproduce; OpenVINS was still running
  after playback.

Raw timestamp scan found monotonic IMU, image, and camera-info streams. The IMU
stream had `24768` messages at about 200 Hz with a maximum observed gap of about
`15 ms`. The image stream had some frame gaps up to about `100 ms`.

After the per-instance TF update, rerun this replay with
`head_d435i_openvins_phase2.launch.py start_camera:=false use_sim_time:=true`
and check that regenerated marker observations use `target_frame: head_imu`,
OpenVINS odometry uses `child_frame_id: head_imu`, and TF contains
`marker_map -> head_imu -> head_cam0`.

## Commit Notes

`docker_ws/src/open_vins` has been converted from the broken parent-repo gitlink
into a lean vendored source tree so the Phase 2 OpenVINS edits can be committed
with the parent repo. The vendored copy should include build-critical
`ov_core`, `ov_init`, `ov_msckf`, `config`, and top-level license/readme
metadata. Large optional OpenVINS datasets/evaluation/docs assets are ignored.

Generated overlay directories must not be committed:

```bash
docker_ws/build_overlay/
docker_ws/install_overlay/
docker_ws/log_overlay/
```
