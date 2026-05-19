# Validation Handoff: Host Pipeline With Live Jetson Cameras

Timestamp: 2026-05-20 00:39 Europe/Copenhagen

Worktree:

```bash
/home/asger/Drive/AAU/P8/grasping/mvp-agent-worktrees/agent-h-validation-rviz
branch: tonight/agent-h-validation-rviz
```

This file summarizes what was validated after the Jetson cameras were started and physically "jerked" to initialize OpenVINS. No Jetson-side files were changed during this validation pass. No hand or wrist movement was commanded.

## Current Result

The host-side integrated launch starts cleanly in hardware-safe mode, but the perception pipeline is still blocked at point cloud fusion.

The raw camera point clouds are visible on the host at good rates, and OpenVINS publishes odometry plus dynamic TF under `marker_map`. The blocker is that the RealSense raw point cloud frames are not connected to the OpenVINS `marker_map` TF tree:

```text
marker_map -> head_imu -> head_cam0
marker_map -> arm_imu  -> arm_cam0

head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
arm_d435i_arm_link   -> arm_d435i_arm_depth_frame   -> arm_d435i_arm_depth_optical_frame
```

`pointcloud_fusion` subscribes to the raw clouds:

```text
/head/d435i_head/depth/color/points  frame_id=head_d435i_head_depth_optical_frame
/arm/d435i_arm/depth/color/points    frame_id=arm_d435i_arm_depth_optical_frame
```

It tries to transform them into:

```text
target_frame=marker_map
```

That lookup currently fails, so `/fused_pointcloud` is not published.

## Tests Run

All commands below were run inside the `prosthesis` container, using:

```bash
source /opt/ros/jazzy/setup.bash
source /prosthesis_ws/install/setup.bash
```

### Container Status

```bash
podman ps --format '{{.Names}} {{.Status}}'
```

Observed containers:

```text
orbslam3      Up ~28 hours
segmentation  Up ~50+ minutes
prosthesis    Up ~40+ minutes
```

### Topic Discovery

```bash
ros2 topic list | sort | grep -E "d435|points|odom|tf|ov_msckf|fused|segmentation|hand|twist"
```

Important topics present:

```text
/head/d435i_head/depth/color/points
/arm/d435i_arm/depth/color/points
/head/marker_pose/ov_corrected_odom
/arm/marker_pose/ov_corrected_odom
/ov_msckf/odomimu
/ov_msckf_arm/odomimu
/tf
/tf_static
```

No pre-transformed marker-map point cloud topics were observed:

```text
/head/d435i_head/points_marker_map
/arm/d435i_arm/points_marker_map
```

Those were mentioned in older planning docs as a possible fallback, but they are not available in the current live system.

### Raw PointCloud2 Headers

```bash
ros2 topic echo --once /head/d435i_head/depth/color/points --field header
ros2 topic echo --once /arm/d435i_arm/depth/color/points --field header
```

Observed:

```text
/head/d435i_head/depth/color/points
  frame_id: head_d435i_head_depth_optical_frame

/arm/d435i_arm/depth/color/points
  frame_id: arm_d435i_arm_depth_optical_frame
```

### Raw PointCloud2 Rates

```bash
ros2 topic hz /head/d435i_head/depth/color/points
ros2 topic hz /arm/d435i_arm/depth/color/points
```

Observed:

```text
head cloud: ~15.08 Hz
arm cloud:  ~14.99 Hz
```

This is a pass for raw cloud visibility.

### OpenVINS Odometry

```bash
ros2 topic hz /ov_msckf_arm/odomimu
ros2 topic echo --once /ov_msckf/odomimu --field header
ros2 topic echo --once /ov_msckf/odomimu --field child_frame_id
ros2 topic echo --once /ov_msckf_arm/odomimu --field header
ros2 topic echo --once /ov_msckf_arm/odomimu --field child_frame_id
```

Observed:

```text
/ov_msckf_arm/odomimu rate: ~185.8 Hz

/ov_msckf/odomimu:
  header.frame_id: marker_map
  child_frame_id: head_imu

/ov_msckf_arm/odomimu:
  header.frame_id: marker_map
  child_frame_id: arm_imu
```

This is a pass for odometry availability.

### TF Tree

```bash
ros2 run tf2_tools view_frames
```

Key frames observed:

```text
head_imu:
  parent: marker_map
  rate: ~170 Hz

head_cam0:
  parent: head_imu
  rate: ~170 Hz

arm_imu:
  parent: marker_map
  rate: ~185 Hz

arm_cam0:
  parent: arm_imu
  rate: ~185 Hz

marker_0:
  parent: marker_map
  rate: ~22 Hz

head_d435i_head_color_optical_frame_from_marker:
  parent: marker_map
  rate: ~11.7 Hz

head_d435i_head_color_optical_frame_body_display:
  parent: marker_map
  rate: ~11.7 Hz

head_imu_from_marker:
  parent: marker_map
  rate: ~11.7 Hz

arm_d435i_arm_color_optical_frame_from_marker:
  parent: marker_map
  rate: ~10.6 Hz

arm_d435i_arm_color_optical_frame_body_display:
  parent: marker_map
  rate: ~10.6 Hz

arm_imu_from_marker:
  parent: marker_map
  rate: ~10.6 Hz
```

RealSense static trees observed:

```text
head_d435i_head_link
  -> head_d435i_head_depth_frame
  -> head_d435i_head_depth_optical_frame
  -> head_d435i_head_color_frame
  -> head_d435i_head_color_optical_frame
  -> head_d435i_head_imu_frame

arm_d435i_arm_link
  -> arm_d435i_arm_depth_frame
  -> arm_d435i_arm_depth_optical_frame
  -> arm_d435i_arm_color_frame
  -> arm_d435i_arm_color_optical_frame
  -> arm_d435i_arm_imu_frame
```

Important: the dynamic OpenVINS/marker TF tree and the RealSense static camera-link trees are separate. This is the immediate fusion blocker.

### Direct TF Checks

These fail:

```bash
ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_frame
```

Observed failure:

```text
Could not find a connection because they are not part of the same tree.
```

These are useful and were observed:

```bash
ros2 run tf2_ros tf2_echo head_d435i_head_color_optical_frame head_d435i_head_link
ros2 run tf2_ros tf2_echo arm_d435i_arm_color_optical_frame arm_d435i_arm_link
```

The transform is the RealSense internal color-optical-to-link relation. It was static and roughly:

```text
translation: [0.015, 0.000, 0.000]
rotation xyzw: [0.499, -0.503, 0.495, 0.502]
```

These are also useful and were observed after the frames appeared:

```bash
ros2 run tf2_ros tf2_echo head_cam0 head_d435i_head_color_optical_frame_body_display
ros2 run tf2_ros tf2_echo arm_cam0 arm_d435i_arm_color_optical_frame_body_display
```

The transforms were near the camera pose, with small translations on the order of millimeters. This suggests `head_cam0` / `arm_cam0` and the marker-derived `*_color_optical_frame_body_display` frames are effectively describing the live color optical camera pose in the `marker_map` tree.

### Integrated Host Launch

Hardware-safe launch:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false \
  mia_hand:=false \
  wrist:=false \
  emg:=false \
  haptic:=false \
  camera:=true
```

Observed startup:

```text
pipeline_manager started
pointcloud_fusion started
pointcloud_relay started
odom_to_pose_relay started
segmentation_bridge started
twist_propagation started, active=False
preshaping_service loaded Rust backend, API version=5
proximity_controller started
```

The launch stayed alive until killed by timeout, so composition is basically valid.

Fusion failure:

```text
[pointcloud_fusion] TF not available: head_d435i_head_depth_optical_frame -> marker_map
[pointcloud_fusion] Stats: published=0 (dual=0, cam1_only=0)
```

This confirms the current first gate is TF, not ROS topic discovery.

## Current Gate Map

Treat these as ordered gates. Do not spend time on later gates until the earlier one passes.

### Gate 1: Raw Camera Visibility

Status: PASS

Criteria:

```bash
ros2 topic hz /head/d435i_head/depth/color/points
ros2 topic hz /arm/d435i_arm/depth/color/points
```

Expected:

```text
Both clouds at roughly 15 Hz.
Headers are head_d435i_head_depth_optical_frame and arm_d435i_arm_depth_optical_frame.
```

### Gate 2: OpenVINS / Marker Map

Status: PASS

Criteria:

```bash
ros2 topic hz /ov_msckf/odomimu
ros2 topic hz /ov_msckf_arm/odomimu
ros2 topic echo --once /ov_msckf_arm/odomimu --field header
ros2 topic echo --once /ov_msckf_arm/odomimu --field child_frame_id
```

Expected:

```text
frame_id: marker_map
child_frame_id: head_imu or arm_imu
rates roughly 170-185 Hz
```

### Gate 3: TF Bridge From OpenVINS Camera Frames To Raw RealSense Depth Frames

Status: FAIL / current blocker

Criteria:

```bash
ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_frame
```

Expected:

```text
Continuous transforms, no "not part of the same tree" error.
```

Likely implementation:

Publish host-side static/dynamic bridge transforms that connect:

```text
head_cam0 or head_d435i_head_color_optical_frame_body_display
  -> head_d435i_head_link

arm_cam0 or arm_d435i_arm_color_optical_frame_body_display
  -> arm_d435i_arm_link
```

Once `*_link` is connected, the existing RealSense static tree should provide:

```text
*_link -> *_depth_frame -> *_depth_optical_frame
```

Preferred approach for next agent:

1. Create a small host-side ROS node/package entry that republishes bridge transforms.
2. Use TF lookup to read an existing live transform if available:
   - `head_cam0 -> head_d435i_head_color_optical_frame_body_display`
   - `arm_cam0 -> arm_d435i_arm_color_optical_frame_body_display`
   - or direct marker-map body-display frames
3. Compose that with RealSense `color_optical -> link`.
4. Publish:
   - `head_cam0 -> head_d435i_head_link`
   - `arm_cam0 -> arm_d435i_arm_link`
5. Verify `marker_map -> *_depth_optical_frame` succeeds.

Fast temporary approach:

Launch two `static_transform_publisher` processes with identity-ish transforms from:

```text
head_cam0 -> head_d435i_head_color_optical_frame
arm_cam0  -> arm_d435i_arm_color_optical_frame
```

Then let RealSense static TF connect color optical to link/depth. This is only acceptable as a smoke test because the exact direction and optical/link composition must be checked carefully.

### Gate 4: Fused Cloud Publication

Status: BLOCKED by Gate 3

Criteria:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true

ros2 topic hz /fused_pointcloud
ros2 topic echo --once /fused_pointcloud --field header
```

Expected:

```text
/fused_pointcloud publishes at useful rate.
header.frame_id: marker_map
pointcloud_fusion stats published > 0
```

### Gate 5: Segmentation Input Relay

Status: BLOCKED by Gate 4

Criteria:

```bash
ros2 topic hz /segmentation/input_cloud
ros2 topic echo --once /segmentation/input_cloud --field header
```

Expected:

```text
Matches fused cloud rate/header closely.
```

### Gate 6: Hand Pose / Twist

Status: likely available, not fully validated

Known:

`odom_to_pose_relay` starts and subscribes to `/ov_msckf_arm/odomimu`. Since odometry is live, `/hand_pose`, `/hand_twist`, and `/hand_odom` should be checked next.

Criteria:

```bash
ros2 topic hz /hand_pose
ros2 topic hz /hand_twist
ros2 topic echo --once /hand_pose
ros2 topic echo --once /hand_twist
```

### Gate 7: Twist Propagation Target Selection

Status: BLOCKED by Gate 4 and probably activation state

Known:

`twist_propagation` launches with:

```text
active=False
input_cloud_topic=/fused_pointcloud
segmented_cloud_topic=/segmentation/object_cloud
click_positive_topic=/segmentation/click_positive
segmentation_reset_topic=/segmentation/reset
segmentation_retarget_distance_m=0.10
```

After fused cloud exists, validate it can generate click/reset events when active and when a predicted collision point moves more than 10 cm.

### Gate 8: Segmentation Inference

Status: not validated in this pass

Known:

`segmentation` container is running and `segmentation_ros2_node` starts. The next agent should test this after `/segmentation/input_cloud` exists.

Criteria:

```bash
ros2 topic echo /segmentation/click_positive --once
ros2 topic echo /segmentation/reset --once
ros2 topic hz /segmentation/object_cloud
```

Expected:

```text
New click triggers inference.
Reset clears/retargets when requested.
/segmentation/object_cloud publishes object-only cloud.
```

### Gate 9: Grasp Service / Preshape

Status: service starts, not exercised with real segmented cloud

Known:

`preshaping_service_bridge_node` loaded:

```text
Loaded Rust preshaping backend, API version=5
Preshaping service bridge ready on /grasp_preshaping/compute_grasp
```

Criteria:

```bash
ros2 service list | grep grasp_preshaping
ros2 service type /grasp_preshaping/compute_grasp
```

Then test with a real `/segmentation/object_cloud`.

### Gate 10: Hardware Actuation

Status: intentionally not tested

Do not test Mia hand/wrist until Gates 1-9 are sane. User allowed small wrist/hand movements, but this validation pass did not command hardware.

## Recommended Next-Agent Work

### Task 1: Implement The TF Bridge

Goal:

Make `marker_map -> head_d435i_head_depth_optical_frame` and `marker_map -> arm_d435i_arm_depth_optical_frame` available without modifying Jetson files.

Likely files:

```text
src/prosthesis_launch/launch/pipeline.launch.py
src/prosthesis_launch/launch/twist_propagation_test.launch.py
src/camera/setup.py
src/camera/camera/
config/prosthesis_config.yaml
```

Suggested implementation:

Add a small Python node, for example:

```text
src/camera/camera/openvins_realsense_tf_bridge_node.py
```

Parameters:

```yaml
head_openvins_camera_frame: head_cam0
head_realsense_link_frame: head_d435i_head_link
head_realsense_color_optical_frame: head_d435i_head_color_optical_frame
head_marker_color_frame: head_d435i_head_color_optical_frame_body_display

arm_openvins_camera_frame: arm_cam0
arm_realsense_link_frame: arm_d435i_arm_link
arm_realsense_color_optical_frame: arm_d435i_arm_color_optical_frame
arm_marker_color_frame: arm_d435i_arm_color_optical_frame_body_display
```

Runtime behavior:

1. Wait for OpenVINS/marker frames.
2. Wait for RealSense static transforms.
3. Compute `openvins_camera_frame -> realsense_link_frame`.
4. Publish those as TF, preferably static if the composition is constant for a camera session.
5. Do not republish any child frame that RealSense already owns.

Important TF ownership rule:

Do not publish a second parent for `*_depth_optical_frame`, `*_depth_frame`, `*_color_optical_frame`, or `*_link` unless you are intentionally connecting the root `*_link` into the OpenVINS tree and are sure no other publisher already parents that exact child. The clean target is to provide the missing edge above the RealSense link tree, not to fight the RealSense node for its internal frames.

Potentially correct edge:

```text
head_cam0 -> head_d435i_head_link
arm_cam0  -> arm_d435i_arm_link
```

Validate with:

```bash
ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
```

### Task 2: Wire TF Bridge Into Launch

Add launch toggle:

```text
camera_tf_bridge:=true
```

Default should probably be `true` for tonight because raw cloud fusion depends on it.

Add config section in `config/prosthesis_config.yaml` so frame names can be changed quickly without code edits.

### Task 3: Re-run Fusion Gate

Run:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

In another shell:

```bash
ros2 topic hz /fused_pointcloud
ros2 topic echo --once /fused_pointcloud --field header
ros2 topic hz /segmentation/input_cloud
```

Pass:

```text
/fused_pointcloud publishes.
/segmentation/input_cloud publishes.
frame_id is marker_map.
pointcloud_fusion stats show published > 0.
```

### Task 4: Validate Downstream Perception Without Hardware

Keep hardware disabled:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

Check:

```bash
ros2 topic hz /hand_pose
ros2 topic hz /hand_twist
ros2 topic hz /segmentation/object_cloud
ros2 service list | grep grasp
```

If twist propagation is inactive, activate it through the existing pipeline/EMG topic path if available, or temporarily expose a launch arg for `active:=true` in the integrated launch only for smoke testing.

### Task 5: RViz Evidence

Only after fused cloud exists:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=true mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

RViz fixed frame:

```text
marker_map
```

Must see:

```text
TF tree connected
raw/fused cloud visible
hand pose marker or frame visible
segmentation object cloud after click/inference
```

Take screenshots/videos at this point.

## Known Noise / Non-Blockers

Many warnings like this appear:

```text
[rmw_cyclonedds_cpp]: Failed to parse type hash for topic ...
```

These appear to be cross-version DDS discovery/type-hash noise from the Jetson/host mix. They did not block topic echo, topic hz, TF echo, or launch startup. Do not spend time on this unless a topic is genuinely missing.

## What Not To Do Next

Do not rewrite fusion, segmentation, grasp, EMG, haptic, or force control before the TF bridge and `/fused_pointcloud` gate pass.

Do not tune grasping or haptics before object-only segmentation output exists.

Do not command hand/wrist hardware until perception produces:

```text
/fused_pointcloud
/segmentation/input_cloud
/segmentation/object_cloud
/grasp_preshaping target pose/closures
```

Do not modify Jetson-side existing files for this blocker. The current evidence points to a host-side TF bridge/config issue.

## Fast Resume Commands

Start from the host container:

```bash
podman exec -it prosthesis bash
source /opt/ros/jazzy/setup.bash
source /prosthesis_ws/install/setup.bash
```

Check current first blocker:

```bash
ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
```

If those fail, work on TF bridge.

If those pass, launch pipeline:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

Then:

```bash
ros2 topic hz /fused_pointcloud
ros2 topic hz /segmentation/input_cloud
ros2 topic hz /hand_pose
ros2 topic hz /hand_twist
```

## Bottom Line

The system is close enough that the next useful engineering move is narrow: connect the OpenVINS camera frames to the RealSense raw point cloud frames on the host side. Once that bridge is present, rerun fusion and continue down the gate list in order.
