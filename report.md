# RViz Missing TFs — Diagnosis, Fix, And Verification

## What the previous agent did

Commit `4172faa` ("fix(camera): bridge OpenVINS RealSense TF") added a single
host-side dynamic bridge node:

- Source: `src/camera/camera/openvins_realsense_tf_bridge_node.py`
- Launch wrapper: `src/prosthesis_launch/launch/tf_bridge.launch.py`
- Wired into `src/prosthesis_launch/launch/pipeline.launch.py` via the
  `camera_tf_bridge` argument (default `true`).
- Config block `openvins_realsense_tf_bridge` in
  `config/prosthesis_config.yaml`.

The bridge published two dynamic TF edges at ~15 Hz:

```text
head_cam0 -> head_d435i_head_link
arm_cam0  -> arm_d435i_arm_link
```

That single link is the only thing it added. It validated in the previous session
with `tf2_echo marker_map head_d435i_head_depth_optical_frame` and
`tf2_echo marker_map arm_d435i_arm_depth_optical_frame` (handoff notes in
`.agents/2026-05-20-validation-handoff.md`).

## Full TF tree as it is *supposed* to look

This is the chain the system relies on. Every dotted edge below must be present
for RViz fixed-frame `marker_map` to resolve any of the `head_*` / `arm_*`
camera frames.

```text
marker_map
├── head_imu                                                  (Jetson, OpenVINS, dynamic)
│   └── head_cam0                                             (Jetson, OpenVINS, dynamic)
│       └── head_d435i_head_link                              (HOST bridge, dynamic 15 Hz)  ← FRAGILE
│           ├── head_d435i_head_depth_frame                   (Jetson realsense /tf_static)
│           │   └── head_d435i_head_depth_optical_frame       (Jetson /tf_static)
│           ├── head_d435i_head_color_frame                   (Jetson /tf_static)
│           │   └── head_d435i_head_color_optical_frame       (Jetson /tf_static)
│           ├── head_d435i_head_accel_frame                   (Jetson /tf_static, D435i only)
│           │   └── head_d435i_head_accel_optical_frame       (Jetson /tf_static)
│           ├── head_d435i_head_gyro_frame                    (Jetson /tf_static, D435i only)
│           │   └── head_d435i_head_gyro_optical_frame        (Jetson /tf_static)
│           ├── head_d435i_head_imu_frame                     (Jetson /tf_static)
│           │   └── head_d435i_head_imu_optical_frame         (Jetson /tf_static)
│           └── head_d435i_head_aligned_depth_to_color_frame  (only if align_depth.enable=true; currently false)
│
├── head_d435i_head_color_optical_frame_body_display          (Jetson aruco_marker_pose, dynamic — only when marker visible)
├── head_d435i_head_color_optical_frame_from_marker           (Jetson aruco_marker_pose, dynamic — only when marker visible)
├── head_imu_from_marker                                      (Jetson aruco_marker_pose, dynamic — only when marker visible)
│
├── arm_imu, arm_cam0, arm_d435i_arm_*  (mirror of the head subtree)
│
└── marker_0                                                  (aruco map)
    └── head_d435i_head_color_optical_frame_raw               (Jetson aruco_marker_pose, dynamic — only when marker visible)
        (same for arm under marker_0 -> arm_d435i_arm_color_optical_frame_raw)
```

## Why RViz still showed "a TON" of missing `head_*` / `arm_*` frames

The previous fix only published **one** edge per camera. The entire RealSense
static fan-out below `*_d435i_*_link` (depth, color, accel, gyro, imu frames
plus all optical children — ~10 transforms per camera) was still expected to
arrive from the Jetson over `/tf_static`. In practice that fails for two
reasons:

1. **The bridge may not be running.** `make rviz` only starts the RViz
   container; it does not start the prosthesis pipeline. Without the bridge,
   `*_cam0 -> *_d435i_*_link` is missing and the whole RealSense subtree is
   orphaned from `marker_map`.

2. **Jetson `/tf_static` does not reliably cross DDS to the host.**
   `transient_local` durability is supposed to redeliver to late subscribers,
   but CycloneDDS peer-mode across the Ethernet link is flaky in practice.
   The prosthesis container may see the chain while the RViz container does
   not.

## What was implemented

The bridge node (`openvins_realsense_tf_bridge_node.py`) was extended to
**also broadcast the full nominal D435/D435i static fan-out**, using the
standard extrinsics from `realsense2_description`. This removes the
dependency on Jetson `/tf_static` for visualization.

### Changes

**`src/camera/camera/openvins_realsense_tf_bridge_node.py`**

- Added `StaticTransformBroadcaster` alongside the existing dynamic
  `TransformBroadcaster`.
- On startup, publishes 10 static transforms per camera (20 total):
  - `link -> depth_frame` (zero)
  - `link -> color_frame` (0, 0.015, 0)
  - `link -> accel_frame` (-0.01174, -0.00552, 0.0051)
  - `link -> gyro_frame` (-0.01174, -0.00552, 0.0051)
  - `link -> imu_frame` (zero)
  - each of the above `-> *_optical_frame` with `rpy(-π/2, 0, -π/2)`
- These are sent once via `/tf_static` (proper latched semantics) and also
  re-broadcast on the dynamic `/tf` topic every timer cycle as a safety net
  for containers that miss the transient_local latch.
- New parameter `publish_nominal_static_chain` (default `true`) controls this
  behavior.

**`config/prosthesis_config.yaml`**

- Added `publish_nominal_static_chain: true` under
  `openvins_realsense_tf_bridge`.

**`Makefile.workspace`**

- Added `tonight-tf-full-check` target that enumerates all 22 camera frames
  and verifies they resolve from `marker_map`.

### Nominal extrinsics source

Values come directly from Intel's `realsense2_description` URDF:

- `_d435.urdf.xacro`: depth zero, color offset 0.015 m Y
- `_d435i_imu_modules.urdf.xacro`: accel/gyro offset (-0.01174, -0.00552,
  0.0051) m
- All optical frames: `rpy(-π/2, 0, -π/2)` (standard camera optical
  convention)

These are approximate; the RealSense device has factory-calibrated extrinsics
that override them at runtime on the Jetson. For RViz visualization the
nominal values are sufficient; for pointcloud fusion and grasp planning the
 calibrated Jetson values are still preferred when they arrive.

## Verification

Run inside the `prosthesis` container:

```bash
ros2 launch prosthesis_launch tf_bridge.launch.py config_file:=/prosthesis_ws/config/prosthesis_config.yaml
```

Then in another shell:

```bash
# All 20 static camera frames are present on /tf_static
ros2 topic echo --once /tf_static | grep child_frame_id | sort | uniq

# The local chain resolves even without Jetson OpenVINS running
ros2 run tf2_ros tf2_echo head_d435i_head_link head_d435i_head_depth_optical_frame
ros2 run tf2_ros tf2_echo head_d435i_head_link head_d435i_head_color_optical_frame
ros2 run tf2_ros tf2_echo head_d435i_head_link head_d435i_head_imu_optical_frame
```

Automated test result (2026-05-20):

```text
Static TF children found: 20 / 20
ALL EXPECTED FRAMES PRESENT
```

The full list of frames now published by the host bridge:

```text
head_d435i_head_link -> head_d435i_head_depth_frame
head_d435i_head_link -> head_d435i_head_color_frame
head_d435i_head_link -> head_d435i_head_accel_frame
head_d435i_head_link -> head_d435i_head_gyro_frame
head_d435i_head_link -> head_d435i_head_imu_frame
head_d435i_head_depth_frame -> head_d435i_head_depth_optical_frame
head_d435i_head_color_frame -> head_d435i_head_color_optical_frame
head_d435i_head_accel_frame -> head_d435i_head_accel_optical_frame
head_d435i_head_gyro_frame -> head_d435i_head_gyro_optical_frame
head_d435i_head_imu_frame -> head_d435i_head_imu_optical_frame
arm_d435i_arm_link -> arm_d435i_arm_depth_frame
arm_d435i_arm_link -> arm_d435i_arm_color_frame
arm_d435i_arm_link -> arm_d435i_arm_accel_frame
arm_d435i_arm_link -> arm_d435i_arm_gyro_frame
arm_d435i_arm_link -> arm_d435i_arm_imu_frame
arm_d435i_arm_depth_frame -> arm_d435i_arm_depth_optical_frame
arm_d435i_arm_color_frame -> arm_d435i_arm_color_optical_frame
arm_d435i_arm_accel_frame -> arm_d435i_arm_accel_optical_frame
arm_d435i_arm_gyro_frame -> arm_d435i_arm_gyro_optical_frame
arm_d435i_arm_imu_frame -> arm_d435i_arm_imu_optical_frame
```

## Remaining recommendations

1. **Wire `make rviz` to also start the bridge.** Right now the bridge only
   runs with the pipeline. If you open RViz standalone, the dynamic
   `*_cam0 -> *_link` edge is still missing. Update the `rviz` Makefile
   target or `ros2_ethernet_hello_host.sh` to start
   `tf_bridge.launch.py` as a sidecar.

2. **Fix `anchor_frame_modes` semantics** (lower priority). The current
   `link` mode treats the OpenVINS body-display frame as equal to the
   camera link. A more accurate implementation would compose
   `T_parent_anchor @ T_anchor_link` where `T_anchor_link` is the static
   RealSense color-to-link extrinsic. This is a ~2-line change in
   `_resolve_bridge_transform`.

3. **Calibrated vs nominal values.** If you need the exact factory
   calibrated offsets for the accel/gyro/imu frames on the host, the bridge
   can be extended to subscribe to Jetson `/tf_static`, cache the live
   values, and republish them. For visualization this is not necessary;
   for IMU-based drift analysis it might be.
