# Tonight Integration Runbook

This runbook records the current integrated state after merging agents A-G and
the Agent H launch pass. It is intended for the operator or final integration
agent continuing from the `tonight/agent-h-validation-rviz` worktree.

## Current Branch State

- Branch: `tonight/agent-h-validation-rviz`
- Base integration commit: `c9fa55b Merge agents A-G into agent-h worktree`
- Launch fix commit: `1299fb6 Fix integrated pipeline launch`

## What Changed In The Launch Pass

- `pipeline.launch.py` no longer passes `config/prosthesis_config.yaml` directly
  as a ROS params file. That file mixes ROS parameter sections and flat reference
  sections, so ROS rejects it. The launch file now loads YAML itself and passes
  each node only its own parameter dictionary.
- Full pipeline camera defaults now use the current Jetson topic names:
  - `/head/d435i_head/depth/color/points`
  - `/arm/d435i_arm/depth/color/points`
- Full pipeline launch now has toggles:
  - `camera:=true|false`
  - `mia_hand:=true|false`
  - `wrist:=true|false`
  - `emg:=true|false`
  - `haptic:=true|false`
  - `rviz:=true|false`
- Full pipeline launch now starts haptic bridge/controller when `haptic:=true`.
- `pipeline_manager` release no longer uses unsupported `create_timer(..., one_shot=True)`.
- `grasp_proximity_controller_node.py` now has the missing pipeline-state callback,
  so it can start and gate joint commands during `GRASPING` and `HOLDING`.

## Safe Smoke Commands

From repo root:

```bash
make dev
make segmentation
make shell
```

Inside the `prosthesis` container:

```bash
make build-pkg PKG=prosthesis_launch
make build-pkg PKG=pipeline_manager
make build-pkg PKG=grasp_preshaping
make build-pkg PKG=pointcloud_fusion
make build-pkg PKG=camera
```

No-hardware launch smoke:

```bash
timeout 8s ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=false
```

Perception launch smoke:

```bash
timeout 8s ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

Expected result:

- Launch exits with code `124` from `timeout`.
- Nodes stay up until timeout.
- CycloneDDS type-hash warnings are expected in this mixed Jetson/host setup.
- With `camera:=true`, `pointcloud_fusion` should log the corrected underscore topics.
- If Jetson/OpenVINS TF is not ready, `pointcloud_fusion` may log `TF not available`.

## Hardware Launch Commands

Start the hardware-capable container:

```bash
make up-hw
make shell
```

Inside the container, launch with RViz disabled first:

```bash
ros2 launch prosthesis_launch pipeline.launch.py rviz:=false
```

If a serial device is wrong, override it:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false \
  mia_serial_port:=/dev/ttyUSB0 \
  wrist_serial_port:=/dev/ttyUSB1
```

If you only want perception plus planning, keep hardware off:

```bash
ros2 launch prosthesis_launch pipeline.launch.py \
  rviz:=false mia_hand:=false wrist:=false emg:=false haptic:=false camera:=true
```

## Immediate Validation Checklist

Run these in separate container shells after launch.

Topic visibility:

```bash
ros2 topic list | sort | grep -E 'd435|points|fused|segmentation|twist|hand|pipeline|force|haptic|emg|wrist'
```

Jetson pointclouds:

```bash
ros2 topic hz /head/d435i_head/depth/color/points
ros2 topic hz /arm/d435i_arm/depth/color/points
ros2 topic echo --once /head/d435i_head/depth/color/points --field header
ros2 topic echo --once /arm/d435i_arm/depth/color/points --field header
```

TF:

```bash
ros2 run tf2_ros tf2_echo marker_map head_d435i_head_depth_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_frame
ros2 run tf2_ros tf2_echo marker_map arm_d435i_arm_depth_optical_frame
```

Fusion and segmentation:

```bash
ros2 topic hz /fused_pointcloud
ros2 topic echo --once /fused_pointcloud --field header
ros2 topic hz /segmentation/input_cloud
ros2 topic echo /segmentation/click_positive
ros2 topic hz /segmentation/object_cloud
```

Pipeline state:

```bash
ros2 topic echo /pipeline/state_name
ros2 topic pub --once /emg/confidence std_msgs/msg/Float32 "{data: 0.9}"
ros2 topic pub --once /emg/gesture_label std_msgs/msg/Int32 "{data: 1}"
```

Release smoke:

```bash
ros2 topic pub --once /emg/confidence std_msgs/msg/Float32 "{data: 0.9}"
ros2 topic pub --once /emg/gesture_label std_msgs/msg/Int32 "{data: 3}"
```

## Hardware Safety Checks

Before any hand or wrist motion:

```bash
ros2 node list | grep -E 'mia|command_bridge|force|wrist|pipeline'
ros2 service list | grep -E 'joints|data_streams|set_trajectory|switch'
ros2 topic echo /joint_states
ros2 topic echo data_streams/fingers/forces/data
ros2 topic echo /force_controller/status
```

Very small manual commands only:

```bash
ros2 topic pub --once /thumb_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
ros2 topic pub --once /index_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
ros2 topic pub --once /mrl_pos_ff_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.05]}"
ros2 topic pub --once /wrist/set_position std_msgs/msg/Float64MultiArray "{data: [5.0, 30.0]}"
```

Emergency stop-style commands available tonight:

```bash
ros2 service call /twist_propagation/deactivate std_srvs/srv/Trigger {}
ros2 topic pub --once /segmentation/reset std_msgs/msg/Empty {}
ros2 topic pub --once /haptic_band/motors std_msgs/msg/Float32MultiArray "{data: [0,0,0,0,0,0,0,0]}"
```

## Known Risks

- `config/prosthesis_config.yaml` has duplicate top-level `twist_propagation`
  keys. Python YAML keeps the later flat section. This works for current launch
  because the flat section contains the same runtime keys, but it should be
  cleaned up after the demo.
- EMG classifier currently exposes `REST`, `POWER`, `PINCH`, `OPEN`, and `POINT`.
  There are no distinct stop, force-up, force-down, wrist-positive, or
  wrist-negative classes wired tonight.
- Haptic bridge connects over Bluetooth and sends a connection buzz on first
  successful connection.
- Serial device names are still `/dev/ttyUSB*`; confirm Mia hand vs wrist before
  moving hardware.
- If OpenVINS is restarted, the operator may need to jerk the cameras after boot
  so calibration initializes.

## Evidence To Capture

- RViz showing `/fused_pointcloud`, `/segmentation/object_cloud`,
  `/twist_propagation/predicted_path`, and `/twist_propagation/hit_marker`.
- Terminal output for:
  - `ros2 topic hz /fused_pointcloud`
  - `ros2 topic hz /segmentation/object_cloud`
  - `ros2 topic echo /pipeline/state_name`
  - `ros2 topic echo /force_controller/status`
- Short video of a safe hand/wrist movement if hardware reaches that stage.
