# x86 Full Digital Twin Implementation Report

Date: 2026-05-17
Branch/worktree: `x86-full-digital-twin` at
`/home/asger/Drive/AAU/P8/grasping/wt-x86-full-digital-twin`

## 1. Purpose

This branch moves the full digital-twin perception and grasping pipeline from a
split Jetson-plus-host setup to a single x86 machine. The physical RealSense
D435i cameras are connected to the x86 host, the ArUco marker is physically
visible to the cameras, and all ROS 2 nodes run in local containers.

The end-to-end target is:

1. Local x86 RealSense camera drivers publish both camera streams and point
   clouds.
2. Local x86 marker/OpenVINS-compatible odometry publishes the same odometry
   and TF interfaces that the previous Jetson path published.
3. The digital twin consumes the same camera, odometry, point cloud, hand pose,
   segmentation, and grasp topics as before.
4. RViz uses `world` as fixed frame and can display the camera clouds, fused
   cloud, segmented cloud, TF tree, and Mia hand without "no transform" errors.

The default command is:

```bash
make up-full-digital-twin
```

The default backend is now `DT_PERCEPTION_BACKEND=x86`.

## 2. Top-Level Runtime Commands

### Start

`make up-full-digital-twin` calls:

```bash
bash scripts/start_full_digital_twin_pipeline.sh
```

The script selects a perception backend through `DT_PERCEPTION_BACKEND`:

- `x86` (default): starts `segmentation`, `x86_cameras`, `x86_openvins`,
  `digital_twin`, and `digital_twin_rviz`.
- `jetson`: keeps the old behavior for comparison; it syncs to the Jetson,
  starts Jetson camera/OpenVINS containers, then starts the local host stack.
- `mock`: starts the digital twin without physical cameras and uses the mock
  cloud path.

The relevant x86 branch in the script is:

```bash
services=(segmentation x86_cameras x86_openvins digital_twin digital_twin_rviz)
export DT_CAMERA=true
export DT_GUI=false
export DT_MOCK_EMG="${DT_MOCK_EMG:-false}"
```

### Stop

`make stop-full-digital-twin` calls:

```bash
bash scripts/stop_full_digital_twin_pipeline.sh
```

By default it stops only local compose containers. It stops Jetson containers
only when explicitly requested:

```bash
STOP_JETSON=1 make stop-full-digital-twin
```

### Runtime Check

```bash
make check-full-pipeline
```

This executes `scripts/check_full_pipeline_runtime.sh` inside the
`digital_twin` container and checks the final camera, odometry, fused-cloud,
segmentation, joint-state, and hand command topics.

## 3. Container Architecture

The relevant compose file is `docker/docker-compose.yml`.

### `prosthesis`

This is the main ROS 2 Jazzy image used by most runtime services.

Build context:

```yaml
context: ..
dockerfile: docker/Dockerfile
```

Important runtime properties:

- `network_mode: host` so all containers share the host DDS network.
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`.
- `ROS_DOMAIN_ID=0`.
- X11 socket mounted at `/tmp/.X11-unix`.
- `privileged: true` and `userns_mode: keep-id`.

Important image dependencies added for the x86 setup:

- `ros-jazzy-librealsense2`
- `ros-jazzy-realsense2-camera`
- `ros-jazzy-realsense2-description`
- `ros-jazzy-cyclonedds`
- `ros-jazzy-rmw-cyclonedds-cpp`
- `python3-opencv`
- `python3-scipy`
- `requests`

`python3-opencv` is necessary because the x86 OpenVINS-compatible fallback uses
OpenCV ArUco detection through `cv2.aruco`.

### `segmentation`

This service runs the segmentation inference server from the separate
segmentation image:

```yaml
build:
  context: ../src/segmentation
  dockerfile: ../../docker/Dockerfile.segmentation
```

It exposes HTTP inference on localhost port `5678` by default through host
networking. The ROS 2 `segmentation_ros2_node` in the `digital_twin` container
calls this server at:

```text
http://127.0.0.1:5678/segment
```

### `x86_cameras`

This is the local replacement for the Jetson camera container.

Command:

```bash
source /prosthesis_ws/install/setup.bash &&
timeout ${X86_CONTAINER_LIFETIME:-7200}
ros2 launch sensor_fusion_bringup dual_d435i.launch.py \
  camera_config:=${DT_CAMERA_CONFIG:-d435i_cameras.yaml} \
  enable_pointcloud_neon_fix:=${DT_ENABLE_POINTCLOUD_NEON_FIX:-false}
```

Inputs:

- Physical USB RealSense devices through `/dev:/dev`.
- Udev metadata through `/run/udev:/run/udev:ro`.
- `src/sensor_fusion_bringup/config`, `launch`, and `scripts` live-mounted
  into the installed package path so config and launch edits take effect on the
  next container start without rebuilding the image.

Outputs:

- `/head/d435i_head/color/image_raw`
- `/head/d435i_head/depth/color/points`
- `/head/d435i_head/imu`
- RealSense static/dynamic TFs under the head camera namespace.
- `/arm/d435i_arm/color/image_raw`
- `/arm/d435i_arm/depth/color/points`
- `/arm/d435i_arm/imu`
- RealSense static/dynamic TFs under the arm camera namespace.

Why `enable_pointcloud_neon_fix:=false` on x86:

The RealSense pointcloud filter must explicitly use the color stream for
texturing. The old Jetson path did this through delayed `pointcloud__neon_.*`
parameter writes. The Jazzy x86 RealSense node does not declare those NEON
parameters, but it does declare the standard `pointcloud.stream_filter`
parameter. The x86 fix therefore sets `pointcloud.stream_filter=2` directly in
the launch arguments and leaves the delayed NEON write disabled unless
`DT_ENABLE_POINTCLOUD_NEON_FIX=true` is explicitly supplied.

### `x86_openvins`

This is the local replacement for the Jetson OpenVINS container.

Command:

```bash
source /prosthesis_ws/install/setup.bash &&
timeout ${X86_CONTAINER_LIFETIME:-7200}
ros2 launch sensor_fusion_bringup dual_openvins_phase2.launch.py \
  start_camera:=false \
  rig_mode:=${DT_RIG_MODE:-dual_d435i} \
  use_marker_odometry_fallback:=${DT_USE_MARKER_ODOM_FALLBACK:-true}
```

Inputs:

- Camera images from `x86_cameras`.
- RealSense camera frames and calibration files.
- ArUco marker map config from `sensor_fusion_bringup/config/markers`.

Outputs:

- `/head/marker_pose/*`
- `/arm/marker_pose/*`
- `/ov_msckf_head/odomimu`
- `/ov_msckf_arm/odomimu`
- TF from `marker_map`/`world` to `d435i_head_link`
- TF from `marker_map`/`world` to `d435i_arm_link`

The service depends on `x86_cameras`, but compose dependency only controls
startup order. ROS nodes still tolerate late topics through normal DDS
discovery.

### `digital_twin`

This service runs the main digital-twin launch:

```bash
ros2 launch prosthesis_launch digital_twin.launch.py \
  camera:=${DT_CAMERA:-true} \
  gui:=${DT_GUI:-false} \
  rviz:=false \
  mock_emg:=${DT_MOCK_EMG:-false}
```

It does not launch cameras itself in the compose path. Instead, it subscribes to
the topics from `x86_cameras` and `x86_openvins`.

### `digital_twin_rviz`

This service starts:

```bash
rviz2 -d /prosthesis_ws/rviz/digital_twin.rviz
```

RViz fixed frame is `world`. That means all displayed sensor and robot frames
must be connected to `world` through TF.

## 4. Camera Launch

File: `src/sensor_fusion_bringup/launch/dual_d435i.launch.py`

This launch file reads `d435i_cameras.yaml` and launches one
`realsense2_camera` include per camera.

Default camera config:

```yaml
cameras:
  head:
    namespace: head
    name: d435i_head
    serial_no: "_336222071386"
  arm:
    namespace: arm
    name: d435i_arm
    serial_no: "_310622071850"
```

Shared RealSense settings:

- Pointcloud enabled.
- Depth aligned to color.
- Gyro enabled.
- Accel enabled.
- IMU unite method set to `2`.
- Depth profile `640x480x6`.
- Color profile `640x480x6`.
- `pointcloud.stream_filter=2`, selecting the color stream for pointcloud
  texture.
- Optional `pointcloud__neon_.stream_filter=2`, available behind
  `DT_ENABLE_POINTCLOUD_NEON_FIX=true` for driver builds that expose the NEON
  pointcloud namespace.
- `decimation_filter_magnitude=4`, reducing depth density before the ROS
  pointcloud is built.

For each camera, `_camera_actions()` creates:

1. A `realsense2_camera/rs_launch.py` include.
2. Optional delayed pointcloud filter parameter commands. On the x86 full
   digital twin these are disabled by default because the Jazzy RealSense
   driver exposes `pointcloud.stream_filter`, not `pointcloud__neon_.*`.
3. Optional external I2C IMU node only when the camera config requests it.

The final x86 rig uses two D435i cameras with built-in IMUs, so no external I2C
IMU should be started.

## 5. OpenVINS-Compatible Marker/Odometry Launch

File: `src/sensor_fusion_bringup/launch/dual_openvins_phase2.launch.py`

This launch can run either the actual per-camera OpenVINS launch files or a
marker-pose fallback. The x86 compose default is:

```text
use_marker_odometry_fallback:=true
```

With fallback enabled, the launch starts:

- `head_marker_pose_phase2.launch.py`
- `arm_marker_pose_phase2.launch.py`
- `head_marker_pose_odometry_bridge`
- `arm_marker_pose_odometry_bridge`

With fallback disabled, it starts the per-camera OpenVINS estimator launch
files:

- `head_d435i_openvins_phase2.launch.py`
- `arm_d435i_openvins_phase2.launch.py`

The fallback path is intentional for this branch because it preserves the
downstream OpenVINS interface without requiring the OpenVINS estimator binary in
the x86 image.

## 6. ArUco Marker Pose Nodes

Files:

- `src/sensor_fusion_bringup/launch/head_marker_pose_phase2.launch.py`
- `src/sensor_fusion_bringup/launch/arm_marker_pose_phase2.launch.py`
- `src/sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`
- `src/sensor_fusion_bringup/config/markers/head_aruco_map.yaml`
- `src/sensor_fusion_bringup/config/markers/arm_aruco_map.yaml`

Each marker launch starts `aruco_marker_pose_node.py` with a side-specific YAML
file.

Head input:

- Image topic: `/head/d435i_head/color/image_raw`
- Camera frame: `d435i_head_color_optical_frame`
- IMU frame: `head_imu`
- Output prefix: `/head/marker_pose`

Arm input:

- Image topic: `/arm/d435i_arm/color/image_raw`
- Camera frame: `d435i_arm_color_optical_frame`
- IMU frame: `arm_imu`
- Output prefix: `/arm/marker_pose`

The node:

1. Subscribes to camera images.
2. Detects configured ArUco markers with OpenCV.
3. Uses the Kalibr camera/IMU calibration file for camera intrinsics and
   `T_cam_imu`.
4. Estimates marker pose using OpenCV solvePnP.
5. Converts marker observation into `marker_map` coordinates.
6. Publishes pose and observation topics under the configured output prefix.

The important output for the x86 fallback is:

- `/head/marker_pose/imu_pose`
- `/arm/marker_pose/imu_pose`

Those are `PoseWithCovarianceStamped` messages in `marker_map`.

## 7. Marker Pose Odometry Bridges

File: `src/sensor_fusion_bringup/scripts/marker_pose_odometry_bridge.py`

This node converts the ArUco marker pose into OpenVINS-compatible outputs. It
is a compatibility shim: downstream code can keep subscribing to
`/ov_msckf_*/odomimu` and listening for camera TFs as if OpenVINS were running.

Head bridge parameters:

- Input: `/head/marker_pose/imu_pose`
- Output odometry: `/ov_msckf_head/odomimu`
- Odometry child frame: `head_imu`
- TF child frame: `d435i_head_link`

Arm bridge parameters:

- Input: `/arm/marker_pose/imu_pose`
- Output odometry: `/ov_msckf_arm/odomimu`
- Odometry child frame: `arm_imu`
- TF child frame: `d435i_arm_link`

For every marker pose message, it publishes:

1. A `nav_msgs/Odometry` message with the same header and pose covariance.
2. A TF transform whose parent is `msg.header.frame_id` (normally
   `marker_map`) and whose child is the configured camera link frame.

The bridge sets high twist covariance values because marker pose does not
provide true inertial velocity. This makes the output usable as a pose/TF
source without pretending to be a high-quality velocity estimator.

The bridge republishes the latest marker-derived camera TF with current
timestamps so downstream nodes do not lose TF just because the marker detector
momentarily pauses. This cache is bounded by `max_cached_tf_age_s`, default
`2.0`, and is configured from compose with `DT_TF_CACHE_MAX_AGE_S`.

## 8. World Frame Contract

The digital twin uses `world` as its main frame.

The marker odometry path uses `marker_map`.

`digital_twin.launch.py` bridges those by publishing a static identity
transform:

```text
marker_map -> world
```

Because TF lookup can invert transforms, this makes the following chains
available:

```text
world <-> marker_map -> d435i_head_link -> head camera optical frames
world <-> marker_map -> d435i_arm_link  -> arm camera optical frames
world -> wrist_link -> hand robot links
```

The live hand pose path is:

```text
marker_map -> d435i_arm_link
marker_map -> world
openvins_hand_tracker computes world -> wrist_link
robot_state_publisher computes wrist_link -> hand links from /joint_states
```

The fallback hand pose path is:

```text
world -> wrist_link
```

That static fallback keeps the hand visible before marker tracking is ready.
Once `openvins_hand_tracker_node` receives the arm camera TF, it republishes a
dynamic `world -> wrist_link` transform based on the arm camera pose and the
configured wrist-camera mounting offset.

## 9. Digital Twin Launch

File: `src/prosthesis_launch/launch/digital_twin.launch.py`

This launch starts the actual digital-twin graph. In the compose path it is run
with:

```text
camera:=true
launch_cameras:=false
rviz:=false
gui:=false
mock_emg:=false by default
```

Important behavior:

- If `camera:=true`, the launch assumes real camera topics already exist.
- If `camera:=true` and `launch_cameras:=true`, it can launch local cameras
  itself, but compose does not use this mode.
- If `camera:=false`, it starts `mock_cloud_publisher`.

With real cameras enabled, the launch sets:

- Scene cloud topic: `/fused_pointcloud`
- Relay input topic: `/fused_pointcloud`
- Head pointcloud topic: `/head/d435i_head/depth/color/points`
- Arm pointcloud topic: `/arm/d435i_arm/depth/color/points`
- Head camera color frame: `d435i_head_color_optical_frame`
- Arm camera color frame: `d435i_arm_color_optical_frame`
- Arm camera link frame: `d435i_arm_link`

The launch creates the nodes described below.

## 10. Pointcloud Merger

File: `src/camera/camera/pointcloud_merger_node.py`

Node name: `pointcloud_merger`

Parameters from launch:

- `cam1_topic=/head/d435i_head/depth/color/points`
- `cam2_topic=/arm/d435i_arm/depth/color/points`
- `output_topic=/fused_pointcloud`
- `target_frame=world`
- `use_cloud_timestamps=false`
- `point_stride=16`
- `publish_rate_hz=5.0`

Subscriptions:

- `/head/d435i_head/depth/color/points`
- `/arm/d435i_arm/depth/color/points`
- TF

Publications:

- `/fused_pointcloud`

Algorithm:

1. Store the latest cloud from each camera.
2. At 5 Hz, try to transform each cloud into `target_frame`.
3. Downsample each incoming camera cloud before TF work by keeping every
   sixteenth point along the row. The RealSense depth stream is also decimated
   by the camera node with `decimation_filter_magnitude=4`.
4. With `use_cloud_timestamps=false`, request the latest transform instead of
   the exact cloud timestamp. This avoids old-camera-stamp TF lookup failures
   when marker-derived camera TFs are being republished at current time.
5. If both transformed clouds are available and their `PointCloud2` fields are
   compatible, it concatenates the raw point data into a single output cloud.
6. If only one transformed cloud is available, publish that transformed cloud
   in `world` rather than publishing an untransformed raw camera cloud.

The desired end state is not fallback. The log message that indicates correct
operation is:

```text
TF chain connected - proper merge active
```

For RViz to display `/fused_pointcloud` in fixed frame `world`, the output
message header must be `world`.

The merged output preserves the incoming RealSense `rgb` field. If the two
input clouds have compatible `PointCloud2` fields, the merger transforms and
concatenates the raw point records, so the packed color bytes survive the
merge. A live x86 validation sample showed `/fused_pointcloud` with fields
`x`, `y`, `z`, and `rgb`, `point_step=20`, `frame_id=world`, and nonzero
varied RGB values. The earlier grey fused view was caused by RViz using an
axis/intensity color transformer, not by the fused topic losing color.

## 11. Pointcloud Relay

File: `src/camera/camera/pointcloud_relay_node.py`

Node name: `pointcloud_relay`

Parameters:

- Input: `/fused_pointcloud`
- Output: `/segmentation/input_cloud`

It republishes the fused cloud unchanged. This isolates segmentation from the
camera/fusion topic name and keeps the segmentation node fixed on
`/segmentation/input_cloud`.

## 12. Segmentation ROS Bridge

File: `src/segmentation/segmentation_bridge/segmentation_ros2_node.py`

Node name: `segmentation_node`

Subscriptions:

- `/segmentation/input_cloud` (`sensor_msgs/PointCloud2`)
- `/segmentation/click_positive` (`geometry_msgs/PointStamped`)
- `/segmentation/click_negative` (`geometry_msgs/PointStamped`)
- `/segmentation/reset` (`std_msgs/Empty`)
- TF, used to transform clicks into the current cloud frame.

Publications:

- `/segmentation/object_cloud` (`sensor_msgs/PointCloud2`)

Parameters:

- `cubeedge`, default `0.05`
- `inference_url`, default `http://127.0.0.1:5678`

Algorithm:

1. Store the latest input cloud as numpy XYZ and RGB arrays.
2. Store positive and negative clicks.
3. On each click, transform the click into the latest cloud frame using TF.
4. POST the cloud arrays and click lists to the HTTP segmentation server at
   `/segment`.
5. Convert the returned boolean mask into a foreground XYZRGB `PointCloud2`.
   The node preserves the selected points' packed `rgb` field so the segmented
   object cloud displays in color instead of greyscale.
6. Publish the result on `/segmentation/object_cloud`.

If TF click transformation fails, the node logs a warning and uses the raw
click coordinates. That is acceptable for debugging but not ideal for precise
operation.

## 13. Click Relay

File: `src/segmentation/segmentation_bridge/demo_click_relay_node.py`

Node name: `click_relay`

Subscriptions:

- `/clicked_point`

Publications:

- `/segmentation/click_positive`

RViz's PublishPoint tool publishes to `/clicked_point`. This relay forwards
those clicks to the segmentation node as positive seeds.

Negative seeds are not generated by this relay. They can be published manually
to `/segmentation/click_negative`.

## 14. OpenVINS Hand Tracker

File: `src/camera/camera/openvins_hand_tracker_node.py`

Node name: `openvins_hand_tracker`

Parameters from launch:

- `world_frame=world`
- `camera_frame=d435i_arm_link`
- `wrist_frame=wrist_link`
- `wrist_cam_tx=-0.04`
- `wrist_cam_ty=-0.01`
- `wrist_cam_tz=0.20`
- `wrist_cam_roll=1.57`
- `wrist_cam_pitch=0.0`
- `wrist_cam_yaw=1.57`
- `publish_rate=15.0`
- `max_cached_tf_age_s=2.0`

Subscriptions:

- TF

Publications:

- TF `world -> wrist_link`

Algorithm:

1. Build a homogeneous transform for the fixed wrist-to-camera mount.
2. Invert that transform to get camera-to-wrist.
3. At the configured publish rate, lookup `world -> d435i_arm_link`.
4. Compute:

   ```text
   world_to_wrist = world_to_camera * camera_to_wrist
   ```

5. Publish the resulting `world -> wrist_link` TF.

This is the key node that makes the RViz hand follow the tracked arm camera.
If it briefly cannot lookup `world -> d435i_arm_link`, it republishes the last
good `world -> wrist_link` transform for up to `max_cached_tf_age_s`. After
that, it stops publishing the stale transform instead of silently keeping an
old hand pose forever.

## 15. Hand Pose Publisher

File: `src/camera/camera/hand_pose_publisher.py`

Node name: `hand_pose_publisher`

Subscriptions:

- TF

Publications:

- `/hand_pose` (`geometry_msgs/PoseStamped`)

Algorithm:

1. At 50 Hz, lookup `world -> wrist_link`.
2. Publish that transform as a `PoseStamped` on `/hand_pose`.

Consumers:

- `twist_propagation_node` estimates hand twist from this pose stream.
- `preshaping_service_bridge_node` consumes it as the current hand pose.

## 16. Robot State Publisher and Joint State Publisher

The digital twin launch starts `robot_state_publisher` with:

```text
mia_hand_digital_twin.urdf.xacro
```

The URDF root is `wrist_link`. It does not create its own `world` link. The
world placement of the hand comes from TF `world -> wrist_link`.

File: `src/pipeline_manager/pipeline_manager/digital_twin_joint_state_publisher.py`

Node name: `digital_twin_joint_state_publisher`

Subscriptions:

- `/thumb_pos_ff_controller/commands`
- `/index_pos_ff_controller/commands`
- `/mrl_pos_ff_controller/commands`

Publications:

- `/joint_states`

Algorithm:

1. Interpret each command topic as a closure scalar in `[0, 1]`.
2. Convert closure scalars to Mia hand joint angles.
3. Compute thumb opposition from the index flexion angle.
4. Publish `sensor_msgs/JointState` at 50 Hz.

This replaces the physical Mia hand hardware interface during digital-twin
tests.

## 17. Cloud Snapshot Node

File: `src/camera/camera/cloud_snapshot_node.py`

Node name: `cloud_snapshot_node`

Subscriptions:

- `/pipeline/state`
- `/segmentation/object_cloud`

Publications:

- `/segmentation/object_cloud_snapshot`
- `/segmented_object_cloud`

Behavior:

- In IDLE/SEGMENTING, it forwards the live segmented cloud.
- On PLANNING/APPROACHING, it deep-copies and freezes the latest object cloud.
- During GRASPING/HOLDING, it keeps publishing the frozen cloud.
- On RELEASING/IDLE, it clears the snapshot and resumes live forwarding.

This prevents the object cloud from changing or disappearing when the digital
hand approaches and occludes the object.

## 18. Twist Propagation Node

File: `src/twist_propagation/twist_propagation/twist_propagation_node.py`

Node name: `twist_propagation`

Parameters from launch:

- `active=true`
- `input_cloud_topic=/fused_pointcloud`

Subscriptions:

- `/hand_pose`
- `/fused_pointcloud`
- `/segmentation/object_cloud`
- TF

Publications:

- `/hand_twist`
- `/segmentation/click_positive`
- `/twist_propagation/status`

Services:

- `/twist_propagation/activate`
- `/twist_propagation/deactivate`

Service clients:

- `/grasp_preshaping/compute_grasp`

Algorithm:

1. Keep a rolling buffer of hand poses.
2. Estimate linear and angular twist from recent pose differences.
3. Build a KDTree from the latest input cloud.
4. Propagate the hand pose forward over a short time horizon.
5. If the propagated pose intersects enough points within the hit threshold,
   publish a positive segmentation click.
6. Wait for `/segmentation/object_cloud` to update.
7. Call `/grasp_preshaping/compute_grasp`.

This node can initiate segmentation automatically based on predicted hand-object
proximity. Manual RViz clicks are also supported through the click relay.

## 19. Preshaping Service Bridge

File: `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`

Node name: `preshaping_service_bridge`

Subscriptions:

- `/hand_pose`
- `/hand_twist`
- `/segmentation/object_cloud`

Publications:

- `/thumb_pos_ff_controller/commands`
- `/index_pos_ff_controller/commands`
- `/mrl_pos_ff_controller/commands`
- `/grasp_preshaping/wrist_pose`
- `/grasp_preshaping/target_hand_pose`
- `/grasp_preshaping/target_finger_closures`
- `/grasp_preshaping/grasp_type`

Service:

- `/grasp_preshaping/compute_grasp`

Implementation:

1. Dynamically loads the Rust shared library `libgrasp_preshaping.so`.
2. Stores the latest hand pose, hand twist, and object point cloud.
3. On service call, converts the latest ROS messages into FFI structs.
4. Looks up camera positions in TF using configured camera frames.
5. If no camera TF is available, estimates a fallback camera position from the
   hand pose.
6. Calls the Rust planner through `grasp_preshaping_compute`.
7. Publishes planner outputs as ROS topics.

In the x86 digital-twin launch, `publish_initial_commands` is set false. That
means the bridge publishes target planner topics, and the digital-twin position
controller performs the actual command publication for the hand model.

## 20. Segmented Cloud Grasp Trigger

File:
`src/pipeline_manager/pipeline_manager/segmented_cloud_grasp_trigger.py`

Node name: `segmented_cloud_grasp_trigger`

Subscriptions:

- `/segmentation/object_cloud`

Service clients:

- `/grasp_preshaping/compute_grasp`

Parameters:

- `cloud_topic`, default `/segmentation/object_cloud`
- `compute_service`, default `/grasp_preshaping/compute_grasp`
- `min_points`, default `1`
- `debounce_s`, default `1.0`
- `service_wait_s`, default `0.2`

Behavior:

1. Listen for segmented object clouds.
2. Ignore empty clouds.
3. Debounce repeated segmented-cloud messages so one segmentation result does
   not produce an uncontrolled burst of planner calls.
4. Call the grasp preshaping service as soon as a non-empty segmented cloud is
   received.
5. Log whether the service call completed successfully.

This node makes the manual click path and the collision-click path converge on
the same behavior: once segmentation publishes an object cloud, the grasp
planner runs automatically. The planner is still allowed to fail because of
object geometry or missing inputs; the important integration contract is that
the service is invoked.

## 21. Digital Twin Position Grasp Controller

File:
`src/pipeline_manager/pipeline_manager/digital_twin_position_grasp_controller.py`

Node name: `digital_twin_position_grasp_controller`

Subscriptions:

- `/grasp_preshaping/target_finger_closures`
- `/pipeline/state`

Publications:

- `/thumb_pos_ff_controller/commands`
- `/index_pos_ff_controller/commands`
- `/mrl_pos_ff_controller/commands`

Behavior:

1. Receives planner target closures `[thumb, index, mrl]`.
2. Clamps each closure to `[0, 1]`.
3. Applies a minimum nonzero closure amount so visible movement is not lost.
4. Publishes command topics at 20 Hz.
5. If `open_on_idle=true`, publishes open-hand commands when the pipeline
   returns to IDLE or RELEASING.

This node is digital-twin-specific because the physical hand can use force
feedback, but the RViz hand model needs direct position commands.

## 21. Pipeline Manager

File: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`

Node name: `pipeline_manager`

States:

- `IDLE=0`
- `SEGMENTING=1`
- `PLANNING=2`
- `APPROACHING=3`
- `GRASPING=4`
- `HOLDING=5`
- `RELEASING=6`

Subscriptions:

- `/emg/gesture_label`
- `/emg/confidence`
- `/grasp_preshaping/grasp_type`
- `/segmentation/object_cloud`

Publications:

- `/pipeline/state`
- `/pipeline/state_name`

Service clients:

- `/grasp_preshaping/compute_grasp`

Main transitions:

1. Grasp EMG gesture in IDLE -> SEGMENTING.
2. Nonempty segmented object cloud in SEGMENTING -> PLANNING.
3. Successful preshaping service response -> APPROACHING.
4. OPEN gesture from active states -> RELEASING.

In the real x86 default, `mock_emg` is false. That means the pipeline will not
auto-cycle unless a real EMG bridge or manual topic publications provide
gesture messages. The perception and RViz display stack can still be validated
without EMG.

## 22. Mock Modes

`DT_PERCEPTION_BACKEND=mock` is still available.

In this mode:

- `DT_CAMERA=false`
- `DT_MOCK_EMG` defaults to true
- `digital_twin.launch.py` starts `mock_cloud_publisher`

Mock cloud publisher:

- File: `src/pipeline_manager/pipeline_manager/mock_cloud_publisher.py`
- Publishes `/camera/depth/color/points`
- Frame: `camera_color_optical_frame`
- Produces a synthetic tabletop and object.

Mock EMG publisher:

- File: `src/pipeline_manager/pipeline_manager/mock_emg_publisher.py`
- Publishes `/emg/gesture_label`
- Publishes `/emg/confidence`
- Alternates grasp and release gestures on a timer.

Mock mode is useful for testing segmentation and grasp state-machine behavior
without physical cameras, but it is not the target of this x86 implementation.

## 23. RViz Configuration

File: `rviz/digital_twin.rviz`

Global fixed frame:

```text
world
```

Enabled displays:

- Grid
- TF
- RobotModel from `/robot_description`
- Head pointcloud `/head/d435i_head/depth/color/points`
- Arm pointcloud `/arm/d435i_arm/depth/color/points`
- Fused cloud `/fused_pointcloud`
- Segmented cloud `/segmentation/object_cloud`
- MarkerArray `/segmentation/seed_markers`
- Axes at `wrist_link`

The pointcloud displays use RGB color where color data exists:

- `FusedCloud`: `Color Transformer: RGB8`
- `SegmentedCloud`: `Color Transformer: RGB8`

This is important because RViz can otherwise display the fused cloud using
axis or intensity coloring even while the underlying `PointCloud2` messages
contain valid packed RGB data.

Tools:

- PublishPoint, publishing to `/clicked_point`

For RViz to be clean, these transforms must resolve:

- `world` to `d435i_head_depth_optical_frame`
- `world` to `d435i_head_color_optical_frame`
- `world` to `d435i_head_link`
- `world` to `d435i_arm_depth_optical_frame`
- `world` to `d435i_arm_color_optical_frame`
- `world` to `d435i_arm_link`
- `world` to `wrist_link`
- `wrist_link` to all Mia hand links from `robot_state_publisher`

## 24. Required TF Tree

The expected frame relationships are:

```text
marker_map
  -> world                       static identity from digital_twin launch
  -> d435i_head_link        marker odometry bridge
       -> head camera frames     RealSense driver
  -> d435i_arm_link          marker odometry bridge
       -> arm camera frames      RealSense driver

world
  -> wrist_link                  openvins_hand_tracker or static fallback
       -> Mia hand links         robot_state_publisher
```

The pointcloud merger needs:

```text
world <- d435i_head_depth_optical_frame
world <- d435i_arm_depth_optical_frame
```

The hand tracker needs:

```text
world <- d435i_arm_link
```

The preshaping service tries to resolve:

```text
world <- d435i_head_color_optical_frame
world <- d435i_arm_color_optical_frame
```

## 25. Topic Contract

Camera and odometry topics:

- `/head/d435i_head/color/image_raw`
- `/head/d435i_head/depth/color/points`
- `/head/d435i_head/imu`
- `/arm/d435i_arm/color/image_raw`
- `/arm/d435i_arm/depth/color/points`
- `/arm/d435i_arm/imu`
- `/ov_msckf_head/odomimu`
- `/ov_msckf_arm/odomimu`

Digital-twin scene topics:

- `/fused_pointcloud`
- `/segmentation/input_cloud`
- `/segmentation/object_cloud`
- `/segmentation/object_cloud_snapshot`
- `/segmented_object_cloud`

Hand and grasp topics:

- `/hand_pose`
- `/hand_twist`
- `/pipeline/state`
- `/pipeline/state_name`
- `/grasp_preshaping/target_hand_pose`
- `/grasp_preshaping/target_finger_closures`
- `/grasp_preshaping/grasp_type`
- `/thumb_pos_ff_controller/commands`
- `/index_pos_ff_controller/commands`
- `/mrl_pos_ff_controller/commands`
- `/joint_states`

RViz/manual input topics:

- `/clicked_point`
- `/segmentation/click_positive`
- `/segmentation/click_negative`
- `/segmentation/reset`

## 26. Debugging Guide

### Containers

List containers:

```bash
cd docker
podman-compose --profile digital_twin ps
```

Logs:

```bash
make logs-digital-twin
podman logs x86_cameras
podman logs x86_openvins
podman logs digital_twin
podman logs digital_twin_rviz
podman logs segmentation
```

### Camera Device Detection

Inside `x86_cameras`:

```bash
ros2 topic list | grep d435i
ros2 topic echo --once /head/d435i_head/color/image_raw sensor_msgs/msg/Image
ros2 topic echo --once /arm/d435i_arm/color/image_raw sensor_msgs/msg/Image
ros2 topic echo --once /head/d435i_head/depth/color/points sensor_msgs/msg/PointCloud2
ros2 topic echo --once /arm/d435i_arm/depth/color/points sensor_msgs/msg/PointCloud2
```

If topics are absent, check:

- USB connection.
- Camera serial numbers in `d435i_cameras.yaml`.
- Permissions to `/dev/bus/usb`.
- RealSense driver logs in `x86_cameras`.

### Marker/Odometry

Inside `x86_openvins` or `digital_twin`:

```bash
ros2 topic echo --once /head/marker_pose/imu_pose geometry_msgs/msg/PoseWithCovarianceStamped
ros2 topic echo --once /arm/marker_pose/imu_pose geometry_msgs/msg/PoseWithCovarianceStamped
ros2 topic echo --once /ov_msckf_head/odomimu nav_msgs/msg/Odometry
ros2 topic echo --once /ov_msckf_arm/odomimu nav_msgs/msg/Odometry
```

If marker pose is absent:

- Confirm image topics are live.
- Confirm the marker is visible and well lit.
- Confirm marker ID and size match `head_aruco_map.yaml` and
  `arm_aruco_map.yaml`.
- Check `x86_openvins` logs for ArUco detection warnings.

### TF Checks

Useful direct checks:

```bash
ros2 run tf2_ros tf2_echo world d435i_head_link
ros2 run tf2_ros tf2_echo world d435i_arm_link
ros2 run tf2_ros tf2_echo world d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo world d435i_arm_depth_optical_frame
ros2 run tf2_ros tf2_echo world wrist_link
```

If camera link transforms exist but optical frame transforms do not, the
RealSense driver static TFs are missing.

If `marker_map -> camera_link` exists but `world -> camera_link` does not,
check the `marker_map -> world` static transform in the `digital_twin`
container.

If `world -> wrist_link` does not move with the arm camera, check
`openvins_hand_tracker` logs and verify `world -> d435i_arm_link`.

### Fused Cloud

```bash
ros2 topic echo --once /fused_pointcloud sensor_msgs/msg/PointCloud2
podman logs digital_twin | grep -i "TF chain"
```

Correct operation should include the pointcloud merger log:

```text
TF chain connected - proper merge active
```

If the merger falls back, RViz may still show a cloud, but it is not a proper
two-camera fused cloud.

### RViz

RViz fixed frame is `world`. If RViz reports no transform from a cloud frame to
`world`, run the matching `tf2_echo` command and work backward:

1. Does marker odometry publish `marker_map -> camera_link`?
2. Does RealSense publish `camera_link -> optical_frame`?
3. Does digital twin publish `marker_map -> world`?
4. Is DDS discovery working across all host-network containers?

## 27. Current Validation Status

Static checks completed before the first live run:

- `bash scripts/check_full_pipeline_contracts.sh`
- `bash scripts/test_final_openvins_contracts.sh`
- `bash scripts/test_final_camera_contracts.sh`
- `bash scripts/test_digital_twin_contracts.sh`
- Compose config generation for the `digital_twin` profile.
- `git diff --check`

Live validation completed on 2026-05-17 after building
`localhost/docker_prosthesis:latest` and starting the stack with:

```bash
make up-full-digital-twin
```

Observed container state:

- `segmentation`: running.
- `x86_cameras`: running.
- `x86_openvins`: running.
- `digital_twin`: running.
- `digital_twin_rviz`: running.

Camera validation:

- Head camera serial `336222071386` detected as `/head/d435i_head`.
- Arm camera serial `310622071850` detected as `/arm/d435i_arm`.
- Both cameras publish color, depth, IMU, and pointcloud topics.
- The final camera profile is `640x480x15` for depth and color, with
  RealSense decimation enabled at magnitude `2`.

Marker/odometry validation:

- `/head/marker_pose/marker_valid` publishes `true`.
- `/arm/marker_pose/marker_valid` publishes `true`.
- `/ov_msckf_head/odomimu` and `/ov_msckf_arm/odomimu` are produced by the
  marker odometry bridge.
- The marker fallback node disables fallback TF once marker tracking becomes
  valid.

TF validation:

- `world -> wrist_link`, `world -> palm`, and `world -> index_fle` resolve.
- `openvins_hand_tracker` logs `OpenVINS camera TF connected - hand follows arm
  camera`; the Mia hand is inserted as a digital twin mounted to the arm camera
  through the fixed wrist-camera transform.
- `marker_camera_optical_tf_node` publishes marker-derived TFs for
  `d435i_head_color_optical_frame`, `d435i_head_depth_optical_frame`,
  `d435i_arm_color_optical_frame`, and `d435i_arm_depth_optical_frame` so
  camera pointcloud frames are connected to `world`.

Pointcloud validation:

- `/head/d435i_head/depth/color/points` publishes.
- `/arm/d435i_arm/depth/color/points` publishes.
- `/fused_pointcloud` publishes with `header.frame_id=world`.
- `pointcloud_merger` logs `TF chain connected - proper merge active`.
- To reduce lag, the merger now uses `point_stride=16` and
  `publish_rate_hz=5.0`, while the RealSense depth filter uses
  `decimation_filter_magnitude=4`. This is intentionally lower density than
  the earlier `point_stride=8` / 6 Hz run that produced approximately `15.6k`
  fused output points per message. The validated lower-density run publishes
  `/fused_pointcloud` at approximately 5 Hz with roughly `1k` to `2k` fused
  output points per message, depending on whether both live camera transforms
  are currently available.
- `/fused_pointcloud` includes an `rgb` field. A live sample showed
  `fields=[x, y, z, rgb]`, `point_step=20`, `frame_id=world`, and hundreds of
  nonzero RGB points with varied samples such as `(125, 134, 125)` and
  `(155, 152, 136)`.

RViz validation:

- `digital_twin_rviz` stays running.
- The RViz container bind-mounts `../rviz` into `/prosthesis_ws/rviz`, so
  changes to `rviz/digital_twin.rviz` are used without rebuilding the image.
- The RViz container mounts the host Xauthority cookie into `/tmp/.Xauthority`
  and uses `XAUTHORITY=/tmp/.Xauthority`; without this mount, the container can
  fail with `qt.qpa.xcb: could not connect to display :0`.
- RViz subscribes to both camera clouds, `/fused_pointcloud`, and
  `/segmentation/object_cloud`.
- `FusedCloud` and `SegmentedCloud` are configured with `Color Transformer:
  RGB8`.
- The invalid `rviz_common/Tools` panel entry was removed, and the segmentation
  object cloud display QoS was changed to volatile.
