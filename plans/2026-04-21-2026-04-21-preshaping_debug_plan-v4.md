# Preshaping Debug Plan v4

## Objective

Fix the preshaping pipeline so that grasp types are correctly classified, collision detection works, finger commands differentiate between grasp types (including thumb opposition), and the system does not drive fingers through objects.

## Root Cause Summary

Three critical issues were identified:

1. **TSDF unit mismatch**: `TSDF_RESOLUTION_MM = 5.0` is used with meter-scale point clouds, collapsing all geometry to grid cell 0.
2. **No camera pose**: The camera array is always empty (`&[]`), disabling inside/outside sign detection.
3. **Same command to all fingers**: The bridge publishes the same `closure_amount` to thumb, index, and MRL regardless of grasp type — no thumb opposition logic.

## Implementation Plan

### Phase 1: Fix TSDF Unit System (CRITICAL — blocks all other work)

- [x] **1.1** Rename `TSDF_RESOLUTION_MM` to `TSDF_RESOLUTION_M` in `c_api.rs:19` and set its value to `0.005` (5mm expressed in meters). Rationale: The point cloud arrives in meters from MuJoCo. The `morton()` function divides by resolution to get grid indices. With 5.0, a 3cm sphere collapses to a single cell. With 0.005, a 3cm sphere spans ~6 cells.
- [x] **1.2** Rename `COLLISION_TOL_MM` to `COLLISION_TOL_M` in `c_api.rs:21` and set its value to `0.005` (5mm = 1 grid cell). Rationale: The TSDF returns distances in grid-cell units, and the collision check compares `distance < collision_tol`. With resolution 0.005m, 1 cell = 0.005m = 5mm. Setting tol to 0.005 means "within 1 cell of the surface."
- [x] **1.3** Remove the `collision_tol = COLLISION_TOL_MM / 1000.0` conversion in `c_api.rs:348`. The constant is now already in meters, so the division by 1000 is wrong. Just use `COLLISION_TOL_M` directly.
- [x] **1.4** Update the `morton()` call to pass `TSDF_RESOLUTION_M` instead of `TSDF_RESOLUTION_MM`.
- [x] **1.5** Update the `get_tsdf()` call in `c_api.rs:339-346` to pass `TSDF_RESOLUTION_M` instead of `TSDF_RESOLUTION_MM`.
- [x] **1.6** Update the `pointcloud_helper.rs` struct field name from `resolution_mm` to `resolution_m` in `Tsdf` (line 167), and update all internal references (`get_distance`, `get_surface_normal`, `get_tsdf`, origin calculation). This is a rename-only change; the math stays the same.
- [x] **1.7** Verify by running `cargo test` in the grasp_preshaping crate. Existing tests use resolution 1.0 with abstract-unit coordinates — they are self-consistent and need no changes.

### Phase 2: Fix No-Collision Default Behavior

- [x] **2.1** In `planner.rs:276-280`, change the `None` branch of `sweep_for_collision` from `closure_amount: 1.0` to `closure_amount: 0.0`. Rationale: When no collision is found during the sweep, the hand should NOT close.
- [x] **2.2** Add a new field to `GraspScoreResult`: `pub found_collision: bool`. Set it to `false` for the `None` case and `true` for the `Some` cases. Rationale: The caller needs to distinguish "no collision found, don't close" from "collision found at closure 0, keep open."
- [x] **2.3** In `c_api.rs`, propagate `found_collision` into `ComputeOutput`. When `found_collision` is false, set `combined_score` to `f64::NEG_INFINITY` so this candidate is never selected as best unless ALL candidates have no collision. Rationale: If some samples find collision and others don't, prefer the ones that found collision. If NO sample finds any collision, output `closure_amount: 0.0` (stay open).
- [x] **2.4** In the bridge node, check if the response indicates no collision was found. If no grasp found collision, publish `0.0` to all controllers. Rationale: The system is activated when the user wants to close the hand, but if no collision is detected at all, the safest behavior is to stay open.

### Phase 3: Per-Finger Grasp Type Commands with Thumb Opposition

This is the core fix for the "1.0, 1.0, 1.0" problem. The planner already evaluates different grasp types with different thumb contact sets (ThumbAbd vs ThumbAdd), but the bridge node ignores the grasp type when publishing commands.

**Key insight**: Thumb opposition in the MuJoCo sim is NOT a separate controller — it is derived from the index flexion angle via `derive_thumb_opposition()` in `mujoco_scene_state_publisher_node.py:95-103`. The mapping is:
- Index angle < `start_index_angle` (-0.06 rad) → thumb opposition at minimum (adducted/lateral)
- Index angle > `end_index_angle` (0.08 rad) → thumb opposition at maximum (abducted/cylindrical)

- [x] **3.1** Extend `GraspComputeResponseFFI` in `c_api.rs:84-90` and `ffi_types.hpp:61-67` to include per-finger closure amounts: `thumb_closure: f64`, `index_closure: f64`, `mrl_closure: f64`.
- [x] **3.2** In `c_api.rs`, add a function `compute_per_finger_output` that takes the best `ScoredGrasp` and produces per-finger closures:
  - **Cylindrical**: thumb = `closure_amount`, index = `closure_amount`, MRL = `closure_amount`. Index flexion is high → thumb opposition is abducted (mode 1 in LUT).
  - **Pinch**: thumb = `closure_amount`, index = `closure_amount`, MRL = 0.0 (locked open). Index flexion is high → thumb opposition is abducted.
  - **Lateral**: thumb = `closure_amount`, index = `closure_amount`, MRL = 0.0 (locked open). Index closure must stay in range where `derive_thumb_opposition` returns minimum opposition (adducted, mode 0 in LUT).
  
  Rationale: The LUT already stores separate contact tables for `ThumbAdd` (mode 0) and `ThumbAbd` (mode 1), so the planner correctly evaluates both. The bridge just needs to respect the result.

- [x] **3.3** Update `ComputeOutput` in `c_api.rs` to carry `thumb_closure`, `index_closure`, and `mrl_closure`.
- [x] **3.4** Update the FFI response writing in `grasp_preshaping_compute` to populate the new fields.
- [x] **3.5** Update `preshaping_service_bridge_node.cpp` to publish per-finger commands:
  - `thumb_cmd_pub_` publishes `thumb_closure`
  - `index_cmd_pub_` publishes `index_closure`
  - `mrl_cmd_pub_` publishes `mrl_closure`
  
  No separate thumb opposition publisher is needed — the MuJoCo scene state publisher derives it from the index flexion command automatically via `derive_thumb_opposition()`.

### Phase 4: Real Camera Poses via TF (PRIMARY approach)

The system has two cameras: `front_depth_cam` (world-fixed, TF frame `mujoco_front_depth_cam`) and `wrist_cam` (hand-mounted, TF frame `mujoco_camera_wrist_cam`). Both are published to `/tf` by the MuJoCo TF publisher node. The bridge node must look up these camera positions and pass them through the FFI to the Rust planner.

#### 4A: Extend FFI to carry camera positions

- [x] **4A.1** Add a `CameraPositionFFI` struct to both `c_api.rs` and `ffi_types.hpp`: `{ float x, y, z }`. This represents a single camera position in world frame.
- [x] **4A.2** Extend `GraspComputeRequestFFI` to include a fixed-size camera array: `CameraPositionFFI cameras[4]` and `uint32_t n_cameras`. Using a fixed-size array (max 4 cameras) avoids heap allocation across the FFI boundary. Rationale: The project is multiview-centered and may have multiple depth cameras. A fixed max of 4 is generous for now.
- [x] **4A.3** In `c_api.rs` `compute_from_request`, convert the FFI camera array into a `Vec<Camera>` and pass it to `get_tsdf()` instead of `&[]`. This enables the sign-flipping logic in `pointcloud_helper.rs:434-479`.

#### 4B: Bridge node subscribes to TF and looks up camera positions

- [x] **4B.1** In `preshaping_service_bridge_node.cpp`, add `#include <tf2_ros/buffer.h>` and `#include <tf2_ros/transform_listener.h>`. Create a `tf2_ros::Buffer` and `tf2_ros::TransformListener` as class members.
- [x] **4B.2** Add a parameter `camera_frames` (vector of strings) with default value `{"mujoco_front_depth_cam", "mujoco_camera_wrist_cam"}`. This makes the camera set configurable without recompilation. Rationale: Different scenes may have different camera setups. The multiview nature of the project means camera count may change.
- [x] **4B.3** In `try_handle_direct_request`, before calling the Rust FFI function, look up each camera frame in TF:
  - For each frame in `camera_frames`, call `tf_buffer_->lookupTransform("world", frame, tf2::TimePointZero)`.
  - Extract the translation component as the camera position.
  - Populate the `cameras` array and `n_cameras` in the FFI request.
  - If a TF lookup fails (camera frame not yet available), skip that camera and log a warning.
  Rationale: Using `tf2::TimePointZero` gets the latest available transform. The camera positions are published at 30Hz by the MuJoCo scene state publisher, so they should be available by the time the planner is called.
- [x] **4B.4** If zero cameras are resolved from TF, fall back to estimating the camera position from the hand pose (as a safety net). Log a warning when this happens. The estimated position is: `camera_position = hand_position + hand_rotation * (-0.08, -0.46, 0.10)`. Rationale: It's better to have a rough camera estimate than no camera at all, but this should be a rare fallback.

#### 4C: Verify camera TF pipeline

- [x] **4C.1** In the running sim, verify that `/tf` contains transforms for both `mujoco_front_depth_cam` and `mujoco_camera_wrist_cam`:
  ```
  ros2 topic echo /tf --field transforms | grep -A5 "mujoco_front_depth_cam\|mujoco_camera_wrist_cam"
  ```
- [x] **4C.2** Add a log line in the bridge node that prints the number of cameras resolved and their positions when the planner is called. This makes debugging easy.

### Phase 5: Verification and Testing

- [x] **5.1** Run `cargo test` in the grasp_preshaping crate after Phase 1 changes. All existing tests should pass.
- [x] **5.2** Build the updated Rust library and C++ bridge node.
- [x] **5.3** In the running sim (default pose), call the planner service and verify:
  - The message shows non-trivial closure amounts (not always 1.0).
  - The grasp type varies based on hand position relative to the object.
  - Finger commands differ between thumb, index, and MRL.
  - Camera positions are logged and match the expected TF values.
- [x] **5.4** Move the hand slightly down and verify:
  - Collision is detected during the sweep.
  - Closure amount is less than 1.0.
  - Fingers stop before passing through the object.
- [x] **5.5** Echo `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands` and verify they are no longer all identical.
- [x] **5.6** Verify that when no collision is found (e.g., hand far from object), all commands are 0.0 (hand stays open).
- [x] **5.7** Verify inside/outside detection works: query the planner with the hand positioned so fingers would be inside the object, and confirm the TSDF returns negative distances (sign-flipped).

## Verification Criteria

- [x] `cargo test` passes with renamed constants.
- [x] Default pose: planner returns grasp type with per-finger differentiation.
- [x] Hand near object: closure < 1.0, fingers do not pass through.
- [x] Hand far from object: closure = 0.0, hand stays open.
- [x] Lateral grasp: index flexion low enough that thumb opposition stays adducted.
- [x] Cylindrical/pinch grasp: index flexion drives thumb opposition to abducted.
- [x] TSDF grid cells span ~5mm each (resolution = 0.005m).
- [x] Both `mujoco_front_depth_cam` and `mujoco_camera_wrist_cam` positions are resolved from TF and passed to the planner.
- [x] Inside/outside sign detection is active (camera array is non-empty).

## Potential Risks and Mitigations

1. **TSDF grid becomes too large with 0.005m resolution**
   Mitigation: The ROI clipping (`MIN_TSDF_DIM_M = 0.1`, `MAX_TSDF_DIM_M = 0.3`) limits the grid to at most 60x60x60 cells = 216K entries. This is manageable.

2. **TF not available at planner call time**
   Mitigation: The TF is published at 30Hz and the planner is called on-demand via service. By the time the user triggers the planner, TF should be available. The fallback estimator handles the rare case where TF is not yet ready.

3. **Camera frame names differ between scenes**
   Mitigation: The `camera_frames` parameter is configurable at launch time. Default includes both known cameras. If a scene doesn't have `wrist_cam`, the TF lookup will fail gracefully and that camera is skipped.

4. **Lateral grasp index closure conflicts with thumb opposition coupling**
   Risk: For lateral grasp, we need the thumb adducted, but the index still needs to close. If index closure > 0.08 rad, `derive_thumb_opposition` will start adducting the thumb. Mitigation: The lateral grasp spec uses `ThumbAdd` contacts (mode 0), meaning the planner evaluates with thumb adducted. The index closure for lateral should naturally be in the range that keeps opposition low. Verify during Phase 5.

5. **FFI struct layout change breaks ABI compatibility**
   Mitigation: Both `ffi_types.hpp` and `c_api.rs` must be updated in lockstep. The `GraspComputeRequestFFI` grows by the camera array (4 * 3 * 4 = 48 bytes + 4 bytes for count). The `GraspComputeResponseFFI` grows by 3 * 8 = 24 bytes for per-finger closures. Both sides must be recompiled together.

6. **Multiple cameras vote on inside/outside — conflicting votes**
   Mitigation: The existing voting logic in `pointcloud_helper.rs:475` requires `behind_count > n_cams / 2` AND `inside_votes > behind_count / 2`. With 2 cameras, both must agree a voxel is behind the surface. This is conservative and correct for multiview.

## Alternative Approaches

1. **Pass camera positions as a separate topic instead of TF lookup**: The bridge node could subscribe to a dedicated camera pose topic. Rejected because TF is the standard ROS mechanism for frame lookups and is already published.

2. **Hardcode camera positions from scene XML**: Use the known camera positions from the scene XML as constants. Rejected because it breaks when the scene changes or when using dynamic scenes where cameras move.

3. **Estimate camera from hand pose only (no TF)**: Simpler but inaccurate. Kept as a fallback only (step 4B.4).
