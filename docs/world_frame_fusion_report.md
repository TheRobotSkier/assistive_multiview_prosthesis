# World Frame Fusion Report

## Scope

Host digital twin fusion now uses the final OpenVINS TF tree instead of the old
host ChArUco camera tracker path.

## Changes

- Camera frames now use final namespaced RealSense frames:
  - `head_d435i_head_depth_optical_frame`
  - `head_d435i_head_color_optical_frame`
  - `arm_d435i_arm_link`
  - `arm_d435i_arm_color_optical_frame`
- `marker_map -> world` identity static TF anchors the host grasping frame to
  OpenVINS marker-map output.
- `/fused_pointcloud` is produced in `world`.
- `pointcloud_merger_node` now transforms both input clouds into the target
  frame before concatenating them.
- `openvins_hand_tracker_node` publishes `world -> wrist_link` from the arm
  camera OpenVINS TF plus the fixed wrist-camera mounting offset.
- RViz digital twin config now uses final Jetson topics and `world` fixed frame.

## Interfaces

Inputs:

- `/head/d435i_head/depth/color/points`
- `/arm/d435i_arm/depth/color/points`
- TF from OpenVINS camera frames into `marker_map`

Outputs:

- `/fused_pointcloud` in `world`
- `/segmentation/input_cloud`
- TF `world -> wrist_link`
- RViz hand model via `/joint_states`

## Test

Static checks:

```bash
bash scripts/test_digital_twin_contracts.sh
```

Runtime still needs live Jetson link and OpenVINS topics.
