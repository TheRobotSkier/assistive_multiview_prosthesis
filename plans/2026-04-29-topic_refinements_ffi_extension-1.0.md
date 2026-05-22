# Topic Refinements: Target Pose, Wrist Degrees, and FFI Extension

## Objective

Three changes to the current publishing flow:

1. **`/grasp_preshaping/target_hand_pose`** must publish the **full 6-DOF pose** (position + orientation) from the best scored grasp sample — not the input pose position. This requires extending the FFI boundary with `target_px/py/pz` fields since the Rust planner currently only returns the wrist quaternion, not the target position.

2. **`/grasp_preshaping/wrist_pose`** must change from `geometry_msgs::msg::Pose` (quaternion) to `std_msgs::msg::Float64` (wrist rotation in **degrees**, 0-360). The Rust planner already computes `wrist_rotation` in radians (stored in `SampledPose::wrist_rotation`) but does not return it through the FFI. We need to add a `wrist_rotation_deg` field to the FFI response.

3. **`/grasp_preshaping/target_finger_closures`** remains `Float64MultiArray [thumb, index, mrl]` — no change needed.

---

## Current State Analysis

### What the FFI returns today (`GraspComputeResponseFFI`)

Located at `docker_ws/dev/grasp_preshaping/src/c_api.rs:81-94` and `docker_ws/dev/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp:69-83`:

```
success, closure_amount, combined_score, grasp_type,
thumb_closure, index_closure, mrl_closure,
wrist_qx, wrist_qy, wrist_qz, wrist_qw
```

**Missing**: target position (`target_px/py/pz`) and wrist rotation in degrees (`wrist_rotation_deg`).

### Where the data exists in Rust

- **Target position**: At `c_api.rs:451` the code computes `best_se3 = samples[best_sample_idx].pose.to_se3()` — the SE(3) matrix contains the full target position at indices `[(0,3), (1,3), (2,3)]`. Currently only the rotation (quaternion) is extracted.

- **Wrist rotation**: At `predictor.rs:174` each `SampledPose` stores `wrist_rotation: f64` in **radians** (range `[-FRAC_PI_2, FRAC_PI_2]`). The value is available in `samples[best_sample_idx].wrist_rotation` at `c_api.rs:446` but is never returned through the FFI.

### What the bridge node publishes today

At `preshaping_service_bridge_node.cpp:349-369`:

- `/grasp_preshaping/wrist_pose` — publishes a `geometry_msgs::msg::Pose` with only the quaternion set (no position). **Needs to change to `std_msgs::msg::Float64` in degrees.**
- `/grasp_preshaping/target_hand_pose` — publishes input `pose.position` + FFI wrist quaternion. **Needs to use the FFI target position instead.**

---

## Implementation Plan

### Phase 1 — Extend FFI response struct (Rust side)

File: `docker_ws/dev/grasp_preshaping/src/c_api.rs`

- [ ] **1.1** Add two new fields to `GraspComputeResponseFFI` (line 81-94), inserted **after `mrl_closure` and before `wrist_qx`** to keep related fields grouped:

  ```rust
  /// Target hand position from the best scored grasp sample (world frame).
  pub target_px: f64,
  pub target_py: f64,
  pub target_pz: f64,
  ```

  And one more field for the wrist rotation:

  ```rust
  /// Wrist rotation angle in degrees [0, 360].
  pub wrist_rotation_deg: f64,
  ```

  This must be placed **after the existing fields** but the exact position doesn't matter for `#[repr(C)]` as long as Rust and C++ agree. Recommended order: after `mrl_closure`, add `target_px/py/pz`, then keep `wrist_qx/qy/qz/qw`, then add `wrist_rotation_deg`.

- [ ] **1.2** Add matching fields to the internal `ComputeOutput` struct (line 136-145):

  ```rust
  /// Target hand position from the best scored grasp sample [px, py, pz].
  target_position: [f64; 3],
  /// Wrist rotation in degrees [0, 360].
  wrist_rotation_deg: f64,
  ```

- [ ] **1.3** Populate the new fields in `compute_from_request` (around line 451-466). After computing `best_se3`, extract the position:

  ```rust
  let target_position = [best_se3[(0, 3)], best_se3[(1, 3)], best_se3[(2, 3)]];
  ```

  And convert the wrist rotation from radians to degrees:

  ```rust
  let wrist_rotation_deg = samples[best_sample_idx].wrist_rotation.to_degrees();
  ```

  Note: `wrist_rotation` is in `[-FRAC_PI_2, FRAC_PI_2]` (range [-90, 90]). The user wants 0-360. We need to normalize: `(wrist_rotation_deg.rem_euclid(360.0))` to map into [0, 360).

  Add both to the `ComputeOutput` return.

- [ ] **1.4** Update the zero-initialization in `grasp_preshaping_compute` (line 503-515) to include the new fields:

  ```rust
  target_px: 0.0,
  target_py: 0.0,
  target_pz: 0.0,
  wrist_rotation_deg: 0.0,
  ```

- [ ] **1.5** Update the success path in `grasp_preshaping_compute` (line 522-548) to populate the new fields:

  ```rust
  response_ref.target_px = output.target_position[0];
  response_ref.target_py = output.target_position[1];
  response_ref.target_pz = output.target_position[2];
  response_ref.wrist_rotation_deg = output.wrist_rotation_deg;
  ```

  Also update the format string to include `target=({:.4},{:.4},{:.4})` and `wrist_rot={:.1} deg`.

- [ ] **1.6** Bump the API version from `2` to `3` at line 486.

### Phase 2 — Extend FFI response struct (C++ side)

File: `docker_ws/dev/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp`

- [ ] **2.1** Add matching fields to `GraspComputeResponseFFI` (line 69-83), in the **exact same order** as the Rust struct:

  ```cpp
  // Target hand position from the best scored grasp sample (world frame).
  double target_px;
  double target_py;
  double target_pz;
  // Wrist orientation quaternion [qx, qy, qz, qw].
  double wrist_qx;
  double wrist_qy;
  double wrist_qz;
  double wrist_qw;
  // Wrist rotation angle in degrees [0, 360].
  double wrist_rotation_deg;
  ```

### Phase 3 — Update bridge node publishers and logic

File: `docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`

- [ ] **3.1** Change the `wrist_pose_pub_` type from `geometry_msgs::msg::Pose` to `std_msgs::msg::Float64`:

  - Add include: `#include "std_msgs/msg/float64.hpp"` (may already be implicitly available via `float64_multi_array.hpp`, but explicit is better)
  - Change publisher declaration (line 93-94):

    ```cpp
    wrist_pose_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/grasp_preshaping/wrist_pose", 10);
    ```

  - Change member variable type (line 425):

    ```cpp
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wrist_pose_pub_;
    ```

- [ ] **3.2** Update the wrist pose publishing block (line 349-357) to publish degrees:

  ```cpp
  {
    std_msgs::msg::Float64 wrist_msg;
    wrist_msg.data = ffi_response.wrist_rotation_deg;
    wrist_pose_pub_->publish(wrist_msg);
  }
  ```

- [ ] **3.3** Update the target hand pose publishing block (line 359-369) to use FFI target position:

  ```cpp
  {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = ffi_response.target_px;
    target_pose.position.y = ffi_response.target_py;
    target_pose.position.z = ffi_response.target_pz;
    target_pose.orientation.x = ffi_response.wrist_qx;
    target_pose.orientation.y = ffi_response.wrist_qy;
    target_pose.orientation.z = ffi_response.wrist_qz;
    target_pose.orientation.w = ffi_response.wrist_qw;
    target_hand_pose_pub_->publish(target_pose);
  }
  ```

- [ ] **3.4** Update the service response message (line 387-396) to include the target position and wrist rotation degrees.

### Phase 4 — Build verification

- [ ] **4.1** Build with `colcon build --packages-select grasp_preshaping` inside Docker. Both the Rust cdylib and the C++ node must rebuild cleanly.

---

## Updated Topic Map

| Topic | Message Type | Content |
|---|---|---|
| `/grasp_preshaping/wrist_pose` | `std_msgs/Float64` | Wrist rotation in degrees [0, 360] |
| `/grasp_preshaping/target_hand_pose` | `geometry_msgs/Pose` | Full 6-DOF pose from best grasp (position from planner + wrist quaternion) |
| `/grasp_preshaping/target_finger_closures` | `Float64MultiArray` | Full `[thumb, index, mrl]` closure 0.0-1.0 |
| `/grasp_preshaping/grasp_type` | `Int32` | 1=cylindrical, 2=pinch, 3=lateral |

---

## Verification Criteria

- [ ] `GraspComputeResponseFFI` has identical field layout in Rust (`c_api.rs`) and C++ (`ffi_types.hpp`)
- [ ] API version bumped to 3
- [ ] `/grasp_preshaping/wrist_pose` publishes `Float64` in degrees [0, 360]
- [ ] `/grasp_preshaping/target_hand_pose` publishes the planner's predicted target position (not the input pose)
- [ ] `/grasp_preshaping/target_finger_closures` unchanged — still `[thumb, index, mrl]`
- [ ] `colcon build --packages-select grasp_preshaping` succeeds

---

## Potential Risks and Mitigations

1. **FFI struct layout mismatch** — the most critical risk. If Rust and C++ fields are not in the same order, memory will be misinterpreted.
   - Mitigation: Place fields in identical order in both files. The plan specifies the exact order.

2. **Wrist rotation range** — `wrist_rotation` in Rust is `[-FRAC_PI_2, FRAC_PI_2]` radians = [-90, 90] degrees. The user wants [0, 360].
   - Mitigation: Use `rem_euclid(360.0)` in Rust to normalize. For the current range this maps -90 → 270, 0 → 0, 90 → 90. This is a clean mapping.

3. **Old wrist_pose subscribers break** — any node subscribing to `/grasp_preshaping/wrist_pose` expecting a `Pose` will crash when it receives a `Float64`.
   - Mitigation: This is an intentional API change. The topic name stays the same but the type changes. Document clearly.

---

## Files Modified (Summary)

| File | Changes |
|---|---|
| `docker_ws/dev/grasp_preshaping/src/c_api.rs` | Add `target_px/py/pz`, `wrist_rotation_deg` to FFI response + `ComputeOutput`; populate in `compute_from_request`; update zero-init and success path; bump API version |
| `docker_ws/dev/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp` | Add matching `target_px/py/pz`, `wrist_rotation_deg` fields to C++ struct |
| `docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | Change `wrist_pose_pub_` to `Float64`; use FFI target position for `target_hand_pose`; update publishing blocks |
