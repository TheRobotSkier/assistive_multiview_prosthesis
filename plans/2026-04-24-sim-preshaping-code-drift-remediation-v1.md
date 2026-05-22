# Sim ↔ Preshaping Code Drift Audit & Remediation Plan

## Objective

Identify and fix all forms of code drift (spec drift, documentation drift, interface drift) between the MuJoCo simulation layer and the grasp preshaping pipeline. The goal is a fully consistent, up-to-date ROS 2 network where preshaping works correctly in both the interactive and dynamic simulator modes.

---

## Audit Findings

### Finding 1: Wrist Camera TF Frame Name Mismatch (CRITICAL)

**Source**: `preshaping_service_bridge_node.cpp:57` vs `interactive_system_interface.cpp:543,563`

The preshaping bridge node looks up the wrist camera TF frame as `"mujoco_camera_wrist_cam"`:
```
"mujoco_camera_wrist_cam"
```

But the scene state publisher generates TF frames for cameras using the naming convention `mujoco_camera_{sanitize(name)}` where `name` comes from MuJoCo. The MuJoCo camera in `mia_hand_right_grasp_frame.xml:128` is named `wrist_cam`, so the TF frame would be `mujoco_camera_wrist_cam`.

However, the IMU publisher in `interactive_system_interface.cpp:543,563` publishes with `frame_id = "mujoco_wrist_cam"`. This is inconsistent with the TF frame name.

Additionally, the README (`README.md:107`) documents the wrist frame as `"mujoco_wrist_cam"`, which matches the IMU publisher but NOT the TF frame that the preshaping bridge tries to look up.

**Impact**: The preshaping bridge's TF lookup for the wrist camera will fail at runtime because the TF tree publishes `mujoco_camera_wrist_cam` but the bridge expects `mujoco_camera_wrist_cam` (this happens to match). The IMU data, however, uses `mujoco_wrist_cam` — a different frame ID. Any consumer trying to correlate IMU data with TF will encounter a frame mismatch.

**Priority**: P1 — Directly affects preshaping camera position resolution and TF/IMU consistency.

---

### Finding 2: Covariance Drift — C++ Sim Ignores Rust-Side Fixed Covariance (HIGH)

**Source**: `interactive_system_interface.cpp:450-477` vs `config.rs:22-23` and `c_api.rs:221-229`

The Rust preshaping library has its own fixed covariance constants in `config.rs`:
```rust
FIXED_COV_OMEGA: [0.01, 0.01, 0.01]
FIXED_COV_V: [0.005, 0.005, 0.005]
```

The C++ `InteractiveSystemInterface` computes velocity-scaled covariance with base values:
```cpp
twist_cov_linear_base_ = 0.0005   // vs Rust's 0.005
twist_cov_angular_base_ = 0.001   // vs Rust's 0.01
```

The Rust side's `twist_to_runtime()` in `c_api.rs:221-229` **completely ignores** the covariance data sent in the `TwistWithCovarianceStamped` message and always uses `TwistCovariance::fixed()` (the config.rs constants). This means:

1. The C++ side carefully computes velocity-scaled covariance and publishes it
2. The Rust side discards it entirely and uses hardcoded values

This is a semantic drift: the covariance values in the ROS message are misleading — they suggest the planner uses them, but it doesn't.

**Impact**: The preshaping pipeline is not actually using the velocity-scaled covariance for prediction sampling. The prediction horizon sampling is always based on fixed constants, regardless of hand speed. This reduces grasp quality for moving hands.

**Priority**: P2 — Functional but suboptimal; the planner works but doesn't adapt to hand velocity.

---

### Finding 3: Planner GUI System Interface Publishes Zero Twist (HIGH)

**Source**: `planner_gui_system_interface.cpp:294-314`

The `PlannerGuiSystemInterface` (dynamic simulation mode) publishes a hardcoded zero twist:
```cpp
twist_msg.twist.twist.linear.x = 0.0;
// ... all zeros
twist_msg.twist.covariance[0] = 1e-6;  // angular
twist_msg.twist.covariance[21] = 1e-6; // linear
```

This means the preshaping pipeline always receives zero velocity and minimal covariance when using the planner GUI simulator. The predictor will only sample around the current pose with tiny noise, making the ROI prediction essentially degenerate.

**Impact**: The dynamic simulator's preshaping cannot properly predict future hand positions, limiting it to essentially static grasps.

**Priority**: P2 — The planner GUI mode is a secondary path but should be consistent.

---

### Finding 4: README Documents Stale Old Simulation Workflow (MEDIUM)

**Source**: `README.md:205-243`

The README contains instructions for the "old simulation" that reference a standalone Rust CLI:
```bash
cd src/dev/grasp_preshaping && cargo run -r -- --mode ros --pointcloud-topic /segmented_object_cloud ...
```

However, there is no `main.rs` in the `grasp_preshaping` crate — it only exports a library (`cdylib`). The old standalone CLI mode has been removed entirely. The README still documents this as if it works.

**Impact**: New developers will attempt to follow these instructions and fail. Creates confusion about the current architecture.

**Priority**: P3 — Documentation drift; no runtime impact but wastes developer time.

---

### Finding 5: README Documents `grasp_start` Topic That Doesn't Exist (MEDIUM)

**Source**: `README.md:91-92`

The README documents:
> `/mujoco/grasp_start` — reserved topic for triggering the autonomous grasping algorithm. Published when the simulation starts; no messages are sent yet.

No code in the codebase publishes or subscribes to `/mujoco/grasp_start`. This topic was apparently planned but never implemented. The README describes it as if it exists.

**Impact**: Misleading documentation; consumers expecting this topic will find nothing.

**Priority**: P3 — Documentation drift.

---

### Finding 6: README Unverified Examples Warning (LOW)

**Source**: `README.md:149`

```
### ASGER: I have not actually tested these examples! I did not have time.
```

The README contains an explicit note that the ROS topic examples (terminal and Python) are untested. This is a code quality issue — the examples may or may not work.

**Impact**: Potential confusion for users following the documented examples.

**Priority**: P4 — Low urgency but should be addressed.

---

### Finding 7: Hand Pose Alias Topic `/hand_pose` Lacks Namespacing (LOW)

**Source**: `interactive_system_interface.cpp:147` and `planner_gui_system_interface.cpp:124`

Both system interfaces publish the same hand pose data on two topics:
- `/mujoco/hand_pose` (namespaced, consistent with other topics)
- `/hand_pose` (alias, no namespace)

The preshaping bridge subscribes to `/hand_pose` (the un-namespaced alias). This works but is fragile — if multiple robots were ever launched, the un-namespaced topic would collide.

**Impact**: Works for single-hand setup but violates naming conventions. Not an immediate bug.

**Priority**: P4 — Design concern for future extensibility.

---

### Finding 8: FFI Struct Documentation vs Implementation Consistency (VERIFIED OK)

**Source**: `ffi_types.hpp` vs `c_api.rs`

The FFI structs are properly kept in sync between C++ and Rust:
- `GraspPoseFFI`: 7 doubles (px, py, pz, qx, qy, qz, qw) — matches
- `GraspTwistFFI`: 6 doubles (lx, ly, lz, ax, ay, az) — matches
- `PointCloudViewFFI`: 7 fields — matches
- `CameraPositionFFI`: 3 floats — matches
- `GraspComputeRequestFFI`: nested structs + cameras[4] + n_cameras — matches
- `GraspComputeResponseFFI`: 7 fields — matches

The `ffi_types.hpp` header comment explicitly warns about the sync requirement, and both sides are aligned.

**Status**: No drift detected.

---

### Finding 9: Point Cloud Frame ID Consistency (VERIFIED OK)

**Source**: `mujoco_scene_state_publisher_node.py:365` vs `preshaping_service_bridge_node.cpp:76`

The scene state publisher publishes the world-frame segmented cloud with `frame_id = world_frame_id` (default `"world"`). The preshaping bridge subscribes to `/segmented_object_cloud` and passes the raw point data to Rust. The Rust side treats the points as world-frame coordinates for TSDF construction. This is consistent.

**Status**: No drift detected.

---

## Implementation Plan

### Phase 1: Critical Fixes (P1)

- [ ] **Task 1.1**: Unify wrist camera frame name across all components.
  - In `preshaping_service_bridge_node.cpp:57`, change `"mujoco_camera_wrist_cam"` to `"mujoco_wrist_cam"` to match the IMU publisher's frame_id and the README documentation.
  - In `mujoco_scene_state_publisher_node.py:310`, the camera TF frame naming for non-primary cameras uses `mujoco_camera_{sanitize(name)}`. Verify that the MuJoCo camera named `wrist_cam` generates TF frame `mujoco_camera_wrist_cam`. If so, either:
    - (a) Update the TF frame name to `mujoco_wrist_cam` by special-casing the wrist camera in `_build_named_camera_frame_map`, or
    - (b) Update the preshaping bridge's `camera_frames_` default to use the TF-published name.
  - Recommended approach: (b) — update the preshaping bridge default to `"mujoco_camera_wrist_cam"` and update the IMU frame_id to match. This avoids special-casing in the TF publisher.
  - **Rationale**: All consumers (TF, IMU, preshaping bridge) must agree on the wrist camera frame name for the TF lookup to succeed.

- [ ] **Task 1.2**: Verify the fix works end-to-end by checking that `tf_buffer_->lookupTransform("world", "mujoco_camera_wrist_cam", ...)` succeeds when the dynamic scene is loaded with TF publishing enabled.
  - **Rationale**: The TF lookup is the runtime mechanism; it must resolve correctly.

### Phase 2: Functional Improvements (P2)

- [ ] **Task 2.1**: Bridge the covariance gap between C++ and Rust.
  - Option A (Recommended): Add covariance fields to `GraspTwistFFI` (or a new `TwistCovarianceFFI` struct) so the C++ bridge passes the actual ROS covariance to Rust. Update `twist_to_runtime()` in `c_api.rs` to use the FFI-provided covariance instead of `TwistCovariance::fixed()`.
  - Option B: Document that the Rust side ignores covariance and remove the velocity-scaled covariance computation from the C++ side to avoid misleading ROS messages.
  - **Rationale**: The velocity-scaled covariance was clearly designed to improve prediction quality. Currently it's wasted effort. Bridging it makes the system work as designed.

- [ ] **Task 2.2**: Implement real twist computation in `PlannerGuiSystemInterface`.
  - The planner GUI simulator should compute twist from pose deltas the same way `InteractiveSystemInterface` does (lines 376-478 of `interactive_system_interface.cpp`).
  - Add the same `hand_twist_prev_pos_`, `hand_twist_prev_quat_`, `hand_twist_prev_sim_time_`, `hand_twist_initialized_` member variables and the numerical differentiation logic.
  - Alternatively, if the planner GUI simulator doesn't have access to sim time, publish a zero twist with the fixed covariance values from `config.rs` and document this limitation.
  - **Rationale**: The planner GUI mode currently sends degenerate twist data, making the prediction pipeline ineffective.

### Phase 3: Documentation Cleanup (P3)

- [ ] **Task 3.1**: Remove or clearly mark the "old simulation" section in `README.md` (lines 205-243).
  - The standalone Rust CLI (`cargo run -- --mode ros`) no longer exists. Either remove this section entirely or add a clear deprecation notice pointing to the current interactive/dynamic simulation workflow.
  - **Rationale**: Stale instructions waste developer time and create confusion.

- [ ] **Task 3.2**: Remove or update the `grasp_start` topic documentation in `README.md` (lines 91-92).
  - Since no code publishes to this topic, remove the documentation entry. If the feature is still planned, add a note like "(planned, not yet implemented)".
  - **Rationale**: Documenting non-existent topics is misleading.

- [ ] **Task 3.3**: Verify and test the Python/terminal examples in the README, then remove the "ASGER: I have not actually tested these examples!" warning (line 149).
  - Run each example against the interactive simulator and confirm they work as documented. Fix any issues found.
  - **Rationale**: Unverified examples erode trust in documentation.

### Phase 4: Design Improvements (P4)

- [ ] **Task 4.1**: Consider namespacing the `/hand_pose` and `/hand_twist` topics.
  - The preshaping bridge subscribes to `/hand_pose` and `/hand_twist`. These should be `/mujoco/hand_pose` and `/mujoco/hand_twist` for consistency with all other topics.
  - Update `preshaping_service_bridge_node.cpp:60,68` to use `/mujoco/hand_pose` and `/mujoco/hand_twist`.
  - The alias publishers (`hand_pose_alias_pub_`) in both system interfaces can be removed, or kept for backward compatibility with a deprecation notice.
  - **Rationale**: Consistent namespacing prevents future collisions and makes the topic graph easier to understand.

- [ ] **Task 4.2**: Add a ROS 2 launch test or integration test that verifies the preshaping service call succeeds.
  - Create a minimal test that launches the interactive simulator with preshaping enabled, waits for all topics to appear, triggers the grasp planner, and checks the service response.
  - **Rationale**: Automated testing prevents future regressions of the kind found in this audit.

---

## Verification Criteria

1. **Frame Consistency**: `ros2 topic echo /mujoco/wrist_cam/imu` shows `frame_id` matching the TF frame published for the wrist camera body. `ros2 run tf2_ros tf2_echo world mujoco_camera_wrist_cam` succeeds.
2. **Preshaping Service**: Calling `ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger` returns `success=true` when the interactive simulator is running with a visible target object.
3. **Topic Graph**: All topics follow the `/mujoco/` namespace convention (except the legacy aliases during transition).
4. **Documentation**: README contains no references to removed features (old CLI, `grasp_start` topic) and all examples are verified.
5. **No Regressions**: `colcon build --packages-select grasp_preshaping mia_hand_mujoco` succeeds without errors or warnings.

## Potential Risks and Mitigations

1. **FFI struct layout change (Task 2.1)**
   Mitigation: Any change to `GraspTwistFFI` requires coordinated updates to both `ffi_types.hpp` and `c_api.rs`. Add a static_assert on sizeof(GraspComputeRequestFFI) in C++ and a corresponding check in Rust. Rebuild both sides simultaneously.

2. **Breaking existing launch configurations (Task 4.1)**
   Mitigation: Keep the alias publishers during a deprecation period. Add a ROS parameter to the preshaping bridge for the topic names so they can be overridden without code changes.

3. **Planner GUI twist computation accuracy (Task 2.2)**
   Mitigation: The planner GUI simulator may not expose sim_time. If not available, use wall-clock time for twist computation and document the approximation.

## Alternative Approaches

1. **Minimal fix only (Tasks 1.1-1.2)**: Fix only the critical wrist camera frame mismatch. This gets preshaping working but leaves the covariance and documentation issues. Lowest effort, highest immediate value.

2. **Full remediation (all tasks)**: Address all findings in priority order. This eliminates all identified technical debt in the sim↔preshaping interaction. Recommended if time permits.

3. **Add abstraction layer**: Instead of fixing individual frame names and topic names, introduce a shared configuration file (YAML) that defines all frame IDs, topic names, and covariance parameters. Both the sim nodes and the preshaping bridge read from this config. This prevents future drift but is a larger refactor.
