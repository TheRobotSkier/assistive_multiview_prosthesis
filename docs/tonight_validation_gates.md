# Tonight Validation Gates

This report describes the current host-side validation path for the live Jetson
camera/OpenVINS setup. The goal is to make each gate observable from Make
targets while RViz can run separately.

## Current Work

The `pc-fusion` branch now includes the host-side OpenVINS-to-RealSense TF
bridge and the integrated hardware-safe pipeline launch. The main change is
that host fusion no longer assumes that the raw D435i cloud frames are already
connected to `marker_map`.

The bridge publishes:

```text
head_cam0 -> head_d435i_head_link
arm_cam0  -> arm_d435i_arm_link
```

That connects the OpenVINS tree:

```text
marker_map -> head_imu -> head_cam0
marker_map -> arm_imu  -> arm_cam0
```

to the RealSense raw cloud trees:

```text
head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
arm_d435i_arm_link   -> arm_d435i_arm_depth_frame   -> arm_d435i_arm_depth_optical_frame
```

The integrated safe launch is:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

This is what `make tonight`, `make tonight-fusion`, and `make twist-launch`
inside the container now use.

`make tonight-tf` and `make tonight-tf-check` use
`prosthesis_launch/tf_bridge.launch.py`. This launch file exists because
`config/prosthesis_config.yaml` is a mixed file: launch files can safely extract
one node's `ros__parameters`, but the full file cannot be passed directly to
`ros2 run --params-file`.

## Gate 0: Raw Jetson Streams

Make target:

```bash
make tonight-raw-check
```

Checks:

```text
/head/d435i_head/depth/color/points
/arm/d435i_arm/depth/color/points
/ov_msckf/odomimu
/ov_msckf_arm/odomimu
/tf
/tf_static
```

Success criteria:

- Raw PointCloud2 headers arrive.
- Head and arm raw cloud rates are nonzero, typically near 15 Hz.
- OpenVINS odometry headers arrive with `marker_map` parent frames.

Barrier symptoms:

- Missing cloud topic: Jetson camera container is not publishing or ROS discovery
  is broken across Ethernet.
- Cloud exists but no odom: OpenVINS is not running or did not initialize.
- Odom exists but no TF: OpenVINS visualizer/TF publishing is disabled or stuck.

## Gate 1: OpenVINS/RealSense TF Bridge

Make targets:

```bash
make tonight-tf
make tonight-tf-check
```

Checks:

```text
marker_map -> head_d435i_head_depth_optical_frame
marker_map -> arm_d435i_arm_depth_optical_frame
```

Success criteria:

- `tf2_echo` resolves both transforms continuously.
- RViz no longer complains that the raw D435i depth optical frames are detached
  from `marker_map`.

Barrier symptoms:

- `Invalid frame ID "marker_map"`: OpenVINS/marker TF is not visible yet.
- `not part of the same tree`: bridge is not running, not built, or using the
  wrong anchor frame names.
- Bridge only uses fallback: OpenVINS body-display anchor frames are absent.

Likely fix type:

- Rebuild `camera` and `prosthesis_launch`.
- Confirm the Jetson publishes `head_cam0`, `arm_cam0`, and the body-display
  frames configured in `config/prosthesis_config.yaml`.

## Gate 2: Pointcloud Fusion

Make targets:

```bash
make tonight-fusion
make tonight-fusion-check
```

Checks:

```text
/fused_pointcloud
/segmentation/input_cloud
```

Success criteria:

- `/fused_pointcloud.header.frame_id == marker_map`.
- `/fused_pointcloud` publishes at a nonzero rate.
- `/segmentation/input_cloud` matches the fused cloud rate.
- Fusion logs show `published>0`, ideally mostly `dual`.

Barrier symptoms:

- `TF not available: head_d435i_head_depth_optical_frame -> marker_map`:
  bridge is not running or not resolving.
- `published=0`: no usable input clouds or TF still missing.
- `cam1_only` only: arm cloud, arm TF, or arm timestamp alignment is failing.

Likely fix type:

- Fix TF bridge config first.
- Then compare actual ROS topic names against `pointcloud_fusion` params.

## Gate 3: Segmentation Input And Inference

Make target:

```bash
make tonight-segmentation-check
```

Host target starts the `segmentation` container first. The in-container target
expects the inference server to already be reachable at `http://127.0.0.1:5678`.

Checks:

- Starts the safe host pipeline.
- Reads one finite XYZ point from `/fused_pointcloud`.
- Publishes that point to `/segmentation/click_positive`.
- Waits for `/segmentation/object_cloud`.

Success criteria:

- The helper prints a click in `marker_map`.
- `/segmentation/object_cloud` arrives with nonzero width and height.

Barrier symptoms:

- No fused cloud: Gate 2 is not passing.
- No object cloud: segmentation container is not running, inference URL is
  wrong, click is on empty background, or the bridge/inference process failed.

Likely fix type:

- Start/restart the segmentation container.
- Use RViz to manually choose a better click point if the automatic sample hits
  a poor part of the scene.

## Gate 4: Twist Propagation

Make target:

```bash
make tonight-twist-check
```

Checks:

- Starts the safe host pipeline.
- Calls `/twist_propagation/activate`.
- Echoes `/twist_propagation/status`.
- Echoes `/segmentation/click_positive` if a trajectory/object hit is found.

Success criteria:

- Activation service returns success.
- Status reports `active=true`.
- When the hand/arm trajectory intersects an object, twist propagation publishes
  a segmentation click.

Barrier symptoms:

- `no_hit`: the stationary scene or current motion does not intersect an object.
- No predicted path: missing `/hand_pose`, `/hand_twist`, or fused cloud.
- Repeated stale-cloud status: fusion is too slow or not publishing.

Likely fix type:

- Physically move the arm/camera so the predicted trajectory intersects an
  object.
- Tune twist hit radius only after fusion and TF are stable.

## Gate 5: Preshaping/Grasp Service

Make target:

```bash
make tonight-grasp-check
```

Checks:

- Starts the safe host pipeline.
- Triggers one segmentation click from the fused cloud.
- Calls `/grasp_preshaping/compute_grasp`.
- Echoes target hand pose, target finger closures, and grasp type.

Success criteria:

- Service has pose, twist, object cloud, and camera TFs.
- Planner publishes target topics.

Barrier symptoms:

- `No pose data received yet`: OpenVINS odom relay is not publishing `/hand_pose`.
- `No twist data received yet`: odom relay has not produced `/hand_twist`.
- `No point cloud data received yet`: segmentation did not publish an object
  cloud.
- `No points in ROI`: ROS plumbing is working, but the segmented object is not
  near the predicted hand ROI. This happened with arbitrary manual clicks during
  validation and is not by itself a wiring failure.

Likely fix type:

- Validate with trajectory-selected segmentation, not arbitrary clicks.
- Fix pipeline sequencing so compute is called only after object cloud, pose,
  and twist are fresh.

## Operational Notes

- `make tonight` and `make twist-launch` inside the container now build the
  focused packages first, then launch the safe integrated pipeline.
- `make tonight-clean` kills only host-side tonight pipeline nodes inside the
  `prosthesis` container. Use it after interrupted launches if stale nodes are
  still visible in `ros2 node list`.
- Host-side `make tonight-*` targets start or reuse the `prosthesis` container
  and then invoke the matching in-container target.
- Host-side `make tonight-segmentation-check` and `make tonight-grasp-check`
  also start the `segmentation` container.
- `make tonight-imu-check` is a side diagnostic for the OpenVINS drift issue. It
  does not launch the pipeline; it lists IMU topics, samples the combined
  RealSense IMU messages, and prints approximate rates.
- Temporary check targets write background launch logs to `/tmp/tonight-*.log`
  inside the `prosthesis` container. The foreground targets still show live
  node logs directly in the terminal.

## Live Validation Results

Observed on 2026-05-20 with the Jetson cameras running and stationary unless
noted:

- `make tonight-build`: passed, 7 focused packages built.
- `make tonight-raw-check`: raw head and arm clouds were visible, both near
  15 Hz; OpenVINS odom headers were visible in `marker_map`.
- `make tonight-imu-check`: combined IMU topics were visible at about 200 Hz.
- `make tonight-tf-check`: passed. Both
  `marker_map -> head_d435i_head_depth_optical_frame` and
  `marker_map -> arm_d435i_arm_depth_optical_frame` resolved.
- `make tonight-fusion-check`: passed. `/fused_pointcloud` published in
  `marker_map` at roughly 6 Hz, and `/segmentation/input_cloud` relayed the
  fused cloud at a matching rate.
- `make tonight-segmentation-check`: passed. The helper published an automatic
  click in `marker_map`, and `/segmentation/object_cloud` returned with
  `width=2234`.
- `make tonight-twist-check`: activation service returned success. Status was
  `active=true`, `state=IDLE`, `reason=no_hit`, with 100 predicted poses. This
  is acceptable for a stationary/non-intersecting trajectory.
- `make tonight-grasp-check`: grasp service was reachable, but returned
  `No points in ROI, cannot score grasps`. This is expected for the automatic
  arbitrary segmentation click and means the next test should use a
  trajectory-selected object near the predicted hand ROI.

## OpenVINS IMU Axis Side Note

Observed behavior to investigate later:

- OpenVINS pose looks stable while the marker is visible.
- When the marker is occluded, odometry can fly off quickly.
- RViz shows a strong vector that may be acceleration.

Hypothesis:

- The D435i IMU axes or gravity convention may be wrong in the OpenVINS config,
  causing gravity to be interpreted as acceleration in another direction.

Quick local scan:

- The current host branch does not contain the full OpenVINS config tree.
- The `jetson_docker` branch contains the relevant files under:

```text
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/
docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/
```

- Both `kalibr_imucam_chain.yaml` files have `T_cam_imu` rotations close to
  identity.
- Both `kalibr_imu_chain.yaml` files use identity `R_IMUtoACC` and
  `R_IMUtoGYRO`.

This is plausible if calibration was generated with the same IMU and camera
frame convention that OpenVINS consumes. It is suspicious if RealSense publishes
the combined `/imu` in a different frame convention than the Kalibr files
assume.

Do not patch this blindly tonight. The next diagnostic should record a
stationary sample:

```bash
make tonight-imu-check
ros2 topic echo --once /head/d435i_head/imu
ros2 topic echo --once /arm/d435i_arm/imu
ros2 run tf2_ros tf2_echo head_d435i_head_imu_frame head_d435i_head_color_optical_frame
ros2 run tf2_ros tf2_echo arm_d435i_arm_imu_frame arm_d435i_arm_color_optical_frame
```

Observed stationary samples on 2026-05-20 while the cameras were turned on and
resting:

```text
/head/d435i_head/imu frame_id=head_d435i_head_imu_optical_frame
linear_acceleration ~= (-2.010, -8.493, -4.590) m/s^2
norm ~= 9.86 m/s^2

/arm/d435i_arm/imu frame_id=arm_d435i_arm_imu_optical_frame
linear_acceleration ~= (-1.520, 9.414, -1.765) m/s^2
norm ~= 9.70 m/s^2

Both combined IMU topics publish at about 200 Hz.
```

The magnitudes are close to gravity, so the IMUs are publishing plausible
stationary acceleration. The axis distribution is the important clue: compare
these vectors against the physical camera orientation and OpenVINS'
`T_cam_imu`/`R_IMUtoACC` convention before changing calibration files.

Then compare the stationary linear acceleration vector against the configured
OpenVINS IMU frame and `T_cam_imu`. If gravity points along an unexpected axis,
fix the Kalibr IMU/camera transform or IMU intrinsic rotation in a small,
single-purpose Jetson-side branch.

Tracked follow-up bead:

```text
mvp-6z1 Investigate OpenVINS IMU axis/gravity drift
```

## Next Work

1. Run `make tonight-build`.
2. Run `make tonight-raw-check`.
3. Run `make tonight-imu-check` if OpenVINS drift is being investigated.
4. Run `make tonight-tf-check` while watching RViz fixed frame `marker_map`.
5. Run `make tonight-fusion-check`.
6. Run `make tonight-segmentation-check` with the segmentation container up.
7. Validate `make tonight-twist-check` by moving the arm/camera so the predicted
   path intersects an object.
8. Use `make tonight-grasp-check` only after the segmentation target is near the
   predicted hand/object ROI.
