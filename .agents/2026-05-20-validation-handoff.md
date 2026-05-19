# Validation Handoff: Host Pipeline With Live Jetson Cameras

Timestamp: 2026-05-20 Europe/Copenhagen

Worktree:

```bash
/home/asger/Drive/AAU/P8/grasping/mvp-agent-worktrees/agent-h-validation-rviz
branch: tonight/agent-h-validation-rviz
```

No Jetson-side files were changed during this pass. Hardware-facing launch flags were kept disabled for validation:

```bash
rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

No hand or wrist movement was commanded.

## Current Result

The original blocker was confirmed and fixed on the host side:

```text
marker_map -> head_imu -> head_cam0
marker_map -> arm_imu  -> arm_cam0

head_d435i_head_link -> head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
arm_d435i_arm_link   -> arm_d435i_arm_depth_frame   -> arm_d435i_arm_depth_optical_frame
```

Raw PointCloud2 messages arrive in the RealSense depth optical frames, while OpenVINS owns `marker_map`. The added bridge node now publishes:

```text
head_cam0 -> head_d435i_head_link
arm_cam0  -> arm_d435i_arm_link
```

With that bridge enabled, `marker_map` resolves to both raw depth optical frames, `/fused_pointcloud` publishes, `/segmentation/input_cloud` receives the fused cloud, segmentation can publish an object cloud from a manual click, and twist propagation can be activated.

The only remaining incomplete end-to-end gate is a successful grasp computation from a trajectory-selected segmentation target. A manual arbitrary segmentation click reached the grasp service, but the planner returned `No points in ROI, cannot score grasps`, which is expected when the segmented cloud is not near the predicted hand ROI.

## Files Changed

```text
src/camera/camera/openvins_realsense_tf_bridge_node.py
src/camera/package.xml
src/camera/setup.py
src/prosthesis_launch/launch/pipeline.launch.py
src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp
config/prosthesis_config.yaml
```

## Validation Commands

All ROS commands were run inside the `prosthesis` container:

```bash
source /opt/ros/jazzy/setup.bash
source /prosthesis_ws/install/setup.bash
```

Focused builds:

```bash
cd /prosthesis_ws
colcon build --packages-select camera prosthesis_launch --symlink-install
colcon build --packages-select grasp_preshaping prosthesis_launch --symlink-install
```

Host launch, hardware disabled:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

Operational note: wrapping `podman exec ... ros2 launch ...` with host-side `timeout` can leave the launched ROS nodes orphaned in the container. Prefer running the launch in the foreground and stopping it with Ctrl-C. If a timed run leaves host nodes behind, inspect with:

```bash
ps -eo pid,ppid,stat,comm,args | grep -E "pipeline_manager|openvins_realsense|pointcloud_|odom_to_pose|segmentation_ros2|twist_propagation|preshaping_service|grasp_proximity|ros2 launch"
```

Then terminate only those host pipeline PIDs inside the `prosthesis` container.

Raw cloud checks:

```bash
ros2 topic echo --once /head/d435i_head/depth/color/points --field header
ros2 topic echo --once /arm/d435i_arm/depth/color/points --field header
ros2 topic hz /head/d435i_head/depth/color/points
ros2 topic hz /arm/d435i_arm/depth/color/points
```

Observed:

```text
/head/d435i_head/depth/color/points frame_id=head_d435i_head_depth_optical_frame
/arm/d435i_arm/depth/color/points   frame_id=arm_d435i_arm_depth_optical_frame
head cloud: about 15 Hz
arm cloud:  about 15 Hz
```

TF checks:

```bash
ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
```

Observed: both resolve continuously after the bridge starts.

Fusion and relay checks:

```bash
ros2 topic echo --once /fused_pointcloud --field header
ros2 topic hz /fused_pointcloud
ros2 topic hz /segmentation/input_cloud
```

Observed:

```text
/fused_pointcloud frame_id=marker_map
/fused_pointcloud rate: about 7.8 Hz
/segmentation/input_cloud rate: about 7.8 Hz
```

Twist propagation checks:

```bash
ros2 topic hz /hand_pose
ros2 topic hz /hand_twist
ros2 service call /twist_propagation/activate std_srvs/srv/Trigger '{}'
ros2 topic echo --once /twist_propagation/status
```

Observed:

```text
/hand_pose frame_id=marker_map, about 114-117 Hz
/hand_twist frame_id=marker_map, about 120-128 Hz
/twist_propagation/status active=true, state=IDLE, reason=no_hit, num_predicted_poses=100
```

`no_hit` is plausible for the stationary validation scene.

Segmentation probe:

```bash
ros2 topic echo --once /fused_pointcloud > /tmp/fused_sample.yaml
# A live point from the fused cloud was republished as a positive click:
ros2 topic pub --once /segmentation/click_positive geometry_msgs/msg/PointStamped ...
ros2 topic echo --once /segmentation/object_cloud --field header
```

Observed object clouds from manual clicks:

```text
frame_id=marker_map
width examples: 4500, 9754, 11673
```

Grasp service probe:

```bash
ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger '{}'
```

Observed after wiring `preshaping_service` params:

```text
Planner called with 2 camera(s): target_frame=marker_map cam0=(0.161,-0.402,0.262)
success=false, message='No points in ROI, cannot score grasps'
```

This validates the ROS plumbing and camera TF lookup path into the grasp service. It does not validate planner success because the segmented cloud was produced from an arbitrary manual point rather than a trajectory collision target near the hand ROI.

## Gate Map

| Gate | Status | Evidence |
|---|---|---|
| Jetson raw clouds visible on host | Pass | both D435i cloud topics present at about 15 Hz |
| OpenVINS pose/TF visible on host | Pass | `marker_map -> *_imu -> *_cam0` present |
| RealSense frames connected to OpenVINS | Pass | bridge resolves `marker_map -> *_depth_optical_frame` |
| Host pointcloud fusion | Pass | `/fused_pointcloud` in `marker_map` at about 7.8 Hz |
| Fused cloud relay to segmentation | Pass | `/segmentation/input_cloud` at about 7.8 Hz |
| Segmentation bridge/inference | Pass | manual click produces `/segmentation/object_cloud` |
| Hand pose/twist relay | Pass | `/hand_pose` and `/hand_twist` in `marker_map` at high rate |
| Twist target selection | Partial pass | activation works; stationary scene produced `no_hit` |
| Grasp service ROS plumbing | Partial pass | service sees object cloud and camera TFs; arbitrary cloud outside ROI |
| Mia hand/wrist/EMG/haptic hardware | Not tested here | hardware flags were disabled |

## Next Agent Plan

1. Rebuild and launch the same safe host pipeline:

```bash
podman exec prosthesis bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  cd /prosthesis_ws &&
  colcon build --packages-select camera prosthesis_launch grasp_preshaping --symlink-install
'

podman exec prosthesis bash -lc '
  source /opt/ros/jazzy/setup.bash &&
  source /prosthesis_ws/install/setup.bash &&
  ros2 launch prosthesis_launch pipeline.launch.py \
    rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
'
```

2. Confirm the perception gates:

```bash
ros2 topic hz /fused_pointcloud
ros2 topic hz /segmentation/input_cloud
ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
```

3. Validate trajectory-selected segmentation instead of arbitrary manual clicks:

```bash
ros2 service call /twist_propagation/activate std_srvs/srv/Trigger '{}'
ros2 topic echo /twist_propagation/status
ros2 topic echo /segmentation/click_positive
ros2 topic echo /segmentation/object_cloud --field header
```

If the hand/camera is stationary and status remains `no_hit`, physically position the setup so the predicted hand trajectory intersects an object, then activate again.

4. Validate grasp success only after step 3 produces an object cloud near the predicted ROI:

```bash
ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger '{}'
ros2 topic echo --once /grasp_preshaping/target_hand_pose
ros2 topic echo --once /grasp_preshaping/target_finger_closures
ros2 topic echo --once /grasp_preshaping/grasp_type
```

5. Enable hardware incrementally:

```bash
# Perception + planning only
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=true mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true

# Then hand only, then wrist only, then EMG/haptic once stop behavior is verified.
```

Keep movements small while validating wrist/hand paths.

## Risks And Notes

- The bridge defaults assume the OpenVINS `*_color_optical_frame_body_display` frames represent camera body/link poses in `marker_map`. Live TF samples supported this: the transform from `*_cam0` to the body-display frame matched the RealSense optical-to-body/link orientation closely.
- If a future Jetson launch changes those body-display frame semantics, switch `anchor_frame_modes` in `config/prosthesis_config.yaml` from `link` to `optical` and re-test `marker_map -> *_depth_optical_frame`.
- The `rmw_cyclonedds_cpp: Failed to parse type hash...` warnings appeared repeatedly but did not block topics, TF, launch, services, fusion, or segmentation.
- Do not treat the current grasp service `No points in ROI` result as a wiring failure until it has been tested with a trajectory-selected object cloud.
