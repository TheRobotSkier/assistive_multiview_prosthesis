# Sprint 1 Report: Host Digital Twin Pipeline

Date: 2026-05-16

## Scope

This sprint focused on the host-side integration path that can be tested before
the second D435i is available:

1. scene cloud -> collision prediction
2. collision hit -> positive segmentation click
3. segmented object cloud -> grasp preshaping
4. preshaping/proximity output -> RViz hand joint state rendering

OpenVINS runtime validation was intentionally deferred until both D435i cameras
are attached.

## Changes

- Fixed the preshaping target hand pose contract:
  - `/grasp_preshaping/target_hand_pose` is now `geometry_msgs/PoseStamped`.
  - The message uses frame `world`.
  - This matches the existing proximity controller and synthetic trajectory
    publisher subscribers.
- Added `pipeline_manager/digital_twin_joint_state_publisher.py`.
  - Subscribes to the same command topics used by the hardware controllers:
    - `/thumb_pos_ff_controller/commands`
    - `/index_pos_ff_controller/commands`
    - `/mrl_pos_ff_controller/commands`
  - Publishes `/joint_states` for `robot_state_publisher` and RViz:
    - `j_thumb_fle`
    - `j_index_fle`
    - `j_mrl_fle`
    - `j_thumb_opp`
  - This replaces the physical Mia hand for digital-twin tests while keeping
    the upstream grasping interfaces unchanged.
- Updated `digital_twin.launch.py`.
  - Collision prediction now runs active by default.
  - With real cameras, twist propagation checks `/fused_pointcloud`.
  - With mock data, relay input uses `/camera/depth/color/points`.
  - The pointcloud relay is now parameterized rather than hard-coded.
  - The digital twin launches the command-driven joint-state publisher by
    default; manual `joint_state_publisher_gui` is optional via `gui:=true`.
- Updated ArUco marker setup.
  - Head and arm marker maps now include marker IDs `0` and `1`.
  - OpenVINS Phase 2 launch files pass `marker_fixed_ids: "0,1"`.
  - ID 1 is currently treated as a bench-test marker-map origin. If IDs 0 and
    1 are used together as a rigid multi-marker map, measure and replace
    marker 1's `T_map_marker`.
- Fixed Makefile test profile invocation for `podman-compose`.
  - `make test`
  - `make test-digital-twin-contracts`
- Fixed Jetson deploy branch selection.
  - `JETSON_BRANCH` now defaults to the current git branch.
  - Jetson sync pushes `HEAD:$(JETSON_BRANCH)` instead of assuming an old
    branch name from another sprint.

## Interfaces

The integrated host path is now:

- `/fused_pointcloud` -> `twist_propagation`
- `/fused_pointcloud` -> `pointcloud_relay` -> `/segmentation/input_cloud`
- `twist_propagation` publishes `/segmentation/click_positive`
- `segmentation_bridge` publishes `/segmentation/object_cloud`
- `preshaping_service_bridge_node` consumes:
  - `/hand_pose`
  - `/hand_twist`
  - `/segmentation/object_cloud`
- `preshaping_service_bridge_node` publishes:
  - `/grasp_preshaping/target_hand_pose` (`PoseStamped`, frame `world`)
  - `/grasp_preshaping/target_finger_closures`
  - `/grasp_preshaping/wrist_pose`
  - `/grasp_preshaping/grasp_type`
- `grasp_proximity_controller_node.py` publishes:
  - `/thumb_pos_ff_controller/commands`
  - `/index_pos_ff_controller/commands`
  - `/mrl_pos_ff_controller/commands`
  - `/wrist/set_position`
- `digital_twin_joint_state_publisher` converts the finger command topics into
  `/joint_states` for the RViz hand model.

## Tests

Passing in Podman on the host:

```bash
make test-digital-twin-contracts
make test
```

`make test` currently passes:

- build
- launch syntax
- preshaping `.so`
- node startup
- twist propagation
- digital twin contracts

## Next Sprint

Once the second D435i is attached:

1. Run `make jetson-list-cameras` and confirm the two serials match
   `src/sensor_fusion_bringup/config/d435i_cameras.yaml`.
2. Run camera-only Jetson streaming and verify host visibility:
   `make jetson-cameras`.
3. Validate marker detection from both camera image topics with ArUco ID 1.
4. Start OpenVINS with `make jetson-openvins` and verify TF/odom topics before
   enabling the full host digital-twin launch.
