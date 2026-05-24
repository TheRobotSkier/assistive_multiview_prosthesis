# Trajectory Flow Integration for Preshaping Node

## Objective

Refine the preshaping bridge node's publishing flow so that:

1. **Immediately** upon a successful grasp computation, the node publishes:
   - The **wrist rotation** (already partially done via `/grasp_preshaping/wrist_pose`)
   - A **reduced (preshaping) closure** amount — a configurable percentage of the full closure — sent directly to the finger controllers. This gives the hand a "pre-grasp" shape while the arm is still approaching.

2. **On separate planner topics**, the node publishes:
   - The **target hand pose** (position + orientation from the planner's best sample)
   - The **full closure** amount for each finger controller
   - The **grasp type** (cylindrical / pinch / lateral)

This enables a downstream **trajectory node** to subscribe to these planner topics, move the arm toward the target pose, and progressively close the hand fully as the arm approaches — while the preshaping node has already rotated the wrist and applied a light closure to signal intent.

---

## Current Architecture Analysis

### What the bridge node does today (`preshaping_service_bridge_node.cpp`)

- Subscribes to `/hand_pose`, `/hand_twist`, `/segmented_object_cloud`
- On `/grasp_preshaping/compute_grasp` service call:
  1. Marshals inputs into FFI structs and calls the Rust `grasp_preshaping_compute`
  2. The Rust backend returns: `thumb_closure`, `index_closure`, `mrl_closure`, `grasp_type`, `wrist_qx/qy/qz/qw`, `closure_amount`, `combined_score`
  3. The bridge publishes **full closure** directly to the three finger controller topics (`/thumb_pos_ff_controller/commands`, etc.)
  4. The bridge publishes wrist orientation to `/grasp_preshaping/wrist_pose`

### Key gap

There is no distinction between "preshaping closure" (sent immediately) and "full closure" (published for a trajectory node to use later). The full closure goes straight to the controllers, and there is no target hand pose published for a trajectory node to consume.

### Relevant source locations

| File | Role |
|---|---|
| `docker_ws/dev/grasp_preshaping/src/config.rs` | Tunable constants — needs new `PRESHAPING_CLOSURE_FRACTION` |
| `docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | Bridge node — needs new publishers and split publishing logic |
| `docker_ws/dev/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp` | FFI response struct — already has all needed fields |
| `docker_ws/dev/grasp_preshaping/src/c_api.rs` | Rust FFI — already returns `wrist_qx/qy/qz/qw`, `grasp_type`, per-finger closures |

The FFI layer already returns everything needed. The changes are confined to `config.rs` and the bridge node.

---

## Implementation Plan

### Phase 1 — Add preshaping closure fraction to config

- [ ] **1.1** Add `PRESHAPING_CLOSURE_FRACTION` constant to `docker_ws/dev/grasp_preshaping/src/config.rs`
  - Type: `f64`, value `0.3` (30% of full closure as the preshape amount)
  - Document: this is the fraction of the planner's computed closure sent immediately to controllers as a "pre-grasp" signal
  - This is a Rust-side constant; the bridge node reads it via a ROS parameter for runtime tunability

### Phase 2 — Add new ROS parameter to the bridge node

- [ ] **2.1** Add a `preshaping_closure_fraction` ROS parameter to `PreshapingServiceBridgeNode`
  - Declared with `declare_parameter<double>("preshaping_closure_fraction", 0.3)`
  - Stored as a member variable `preshaping_closure_fraction_`
  - This allows runtime override via launch argument or CLI, independent of the Rust config constant

### Phase 3 — Add new planner publishers to the bridge node

- [ ] **3.1** Add publisher for target hand pose: `/grasp_preshaping/target_hand_pose` (`geometry_msgs::msg::Pose`)
  - Contains the full 6-DOF target pose (position + orientation) from the best scored sample
  - This is the pose the trajectory node should move the arm toward

- [ ] **3.2** Add publisher for full finger closures: `/grasp_preshaping/target_finger_closures` (`std_msgs::msg::Float64MultiArray`)
  - Publishes a 3-element array: `[thumb_closure, index_closure, mrl_closure]`
  - These are the **full** closure values computed by the planner — not the reduced preshape amount

- [ ] **3.3** Add publisher for grasp type: `/grasp_preshaping/grasp_type` (`std_msgs::msg::Int32`)
  - Publishes the grasp type integer (0=unknown, 1=cylindrical, 2=pinch, 3=lateral) matching `ffi_types.hpp` constants

### Phase 4 — Modify publishing logic in `try_handle_direct_request`

- [ ] **4.1** Change the **immediate controller commands** to use the reduced (preshaping) closure
  - Current: `publish_joint_commands(thumb, index, mrl)` sends full closure
  - New: compute `preshape_thumb = thumb * preshaping_closure_fraction_`, similarly for index and mrl
  - Apply `min_closure_amount_` floor to the preshape values (not the full values)
  - Send the preshape values to the three `/xxx_pos_ff_controller/commands` topics
  - Rationale: the hand should only partially close immediately, giving a "pre-grasp" shape

- [ ] **4.2** Publish the target hand pose on `/grasp_preshaping/target_hand_pose`
  - Construct a `geometry_msgs::msg::Pose` from the FFI response's wrist quaternion (`wrist_qx/qy/qz/qw`)
  - For the position component: use the input hand pose position (the current hand position). The orientation is the planner's target wrist orientation
  - Note: the FFI response currently only returns wrist orientation, not a target position. The target position is implicitly "where the hand needs to be to grasp" — which is the same as the current input pose position with the planned wrist rotation applied. If the planner later provides a target position, this can be extended
  - For now, publish the input pose position + the planned wrist orientation quaternion

- [ ] **4.3** Publish the full finger closures on `/grasp_preshaping/target_finger_closures`
  - Publish the unmodified `thumb_closure`, `index_closure`, `mrl_closure` from the FFI response
  - These are the values the trajectory node will use to close the hand fully when approaching the target

- [ ] **4.4** Publish the grasp type on `/grasp_preshaping/grasp_type`
  - Publish `ffi_response.grasp_type` as an `Int32` message

- [ ] **4.5** Update the service response message string to include the preshape fraction and both sets of closure values for debugging

### Phase 5 — Update the wrist pose publisher

- [ ] **5.1** The existing `/grasp_preshaping/wrist_pose` publisher already publishes the wrist orientation quaternion. Keep this as-is — it remains useful for any node that only needs the wrist rotation
  - No changes needed to this publisher

### Phase 6 — Update CMakeLists.txt and package.xml (if needed)

- [ ] **6.1** Verify that `geometry_msgs`, `std_msgs` are already in `package.xml` dependencies — they are (confirmed at `package.xml:11-14`)
- [ ] **6.2** No new message types needed — all publishers use standard ROS 2 messages (`Pose`, `Float64MultiArray`, `Int32`)

---

## Topic Map (After Implementation)

| Topic | Message Type | Published By | Consumed By | Purpose |
|---|---|---|---|---|
| `/thumb_pos_ff_controller/commands` | `Float64MultiArray` | Bridge node (immediate) | Finger controllers | **Reduced** preshape closure (fraction of full) |
| `/index_pos_ff_controller/commands` | `Float64MultiArray` | Bridge node (immediate) | Finger controllers | **Reduced** preshape closure (fraction of full) |
| `/mrl_pos_ff_controller/commands` | `Float64MultiArray` | Bridge node (immediate) | Finger controllers | **Reduced** preshape closure (fraction of full) |
| `/grasp_preshaping/wrist_pose` | `Pose` | Bridge node (immediate) | Wrist controller / trajectory node | Wrist rotation quaternion (already exists) |
| `/grasp_preshaping/target_hand_pose` | `Pose` | Bridge node (on compute) | **Trajectory node** | Target hand pose (position + planned orientation) |
| `/grasp_preshaping/target_finger_closures` | `Float64MultiArray` | Bridge node (on compute) | **Trajectory node** | Full closure amounts per finger |
| `/grasp_preshaping/grasp_type` | `Int32` | Bridge node (on compute) | **Trajectory node** | Grasp type (1=cylindrical, 2=pinch, 3=lateral) |

### Trajectory Node Integration (Downstream Consumer)

The trajectory node (not part of this plan) would:

1. Subscribe to `/grasp_preshaping/target_hand_pose`, `/grasp_preshaping/target_finger_closures`, `/grasp_preshaping/grasp_type`
2. Plan an arm trajectory to reach `target_hand_pose`
3. As the arm approaches the target (e.g., based on distance or trajectory progress), progressively send the full closure values from `target_finger_closures` to the finger controllers
4. Optionally use `grasp_type` to select different closing profiles (e.g., lateral grasp may close thumb first)

---

## Verification Criteria

- [ ] `preshaping_closure_fraction` parameter is declared with default 0.3 and can be overridden via launch argument
- [ ] On service call, the three finger controller topics receive **reduced** closure values (full × fraction, floored by `min_closure_amount`)
- [ ] `/grasp_preshaping/target_hand_pose` publishes a `Pose` with input position + planned wrist orientation
- [ ] `/grasp_preshaping/target_finger_closures` publishes the **full** `[thumb, index, mrl]` closure values
- [ ] `/grasp_preshaping/grasp_type` publishes the correct integer grasp type
- [ ] `/grasp_preshaping/wrist_pose` continues to publish as before (no regression)
- [ ] Service response message includes preshape fraction and both closure sets for debugging
- [ ] `colcon build --packages-select grasp_preshaping` succeeds
- [ ] No changes to `mia_hand_*` packages or the Rust FFI layer

---

## Potential Risks and Mitigations

1. **Preshape fraction too small to be visible**
   - Mitigation: Default 0.3 (30%) should produce a noticeable pre-grasp shape. The parameter is runtime-tunable, so it can be adjusted without rebuild.

2. **Trajectory node receives stale planner data if compute is called multiple times**
   - Mitigation: Each service call publishes fresh values to all planner topics. The trajectory node should use the latest message (latched or QoS `keep_last(1)`). Consider using `TransientLocal` durability on the planner publishers so a late-joining trajectory node gets the last computed values.

3. **Target hand pose position is just the input pose, not a predicted future position**
   - Mitigation: The planner currently scores grasps at predicted future poses but returns the wrist orientation of the best sample. The target position could be enhanced later to include the best sample's position, but this requires an FFI change. For now, using the input pose position is correct — the wrist rotation is the key output.

4. **Race condition between preshape commands and trajectory node's closure commands**
   - Mitigation: The preshape commands are sent once on service call. The trajectory node should take over finger control after receiving the planner topics. Clear documentation of the handoff protocol in topic names will help.

---

## Alternative Approaches

1. **Use a custom ROS 2 message instead of three separate topics**: A single `GraspPlan` message containing target pose, closures, and grasp type. This guarantees atomicity (all values from the same computation). Trade-off: requires creating a new message package or adding to `mia_hand_msgs` (upstream package — avoid modifying). Using three standard topics is simpler and avoids upstream changes.

2. **Add target position to FFI response**: Extend `GraspComputeResponseFFI` with target position fields from the best sample's pose. This would give the trajectory node a true target position rather than reusing the input pose. Trade-off: requires coordinated changes to Rust FFI, C++ header, and Rust compute logic. Recommended as a **follow-up** after the initial integration is working.

3. **Use a ROS 2 action instead of service + topics**: The compute + trajectory flow could be modeled as an action (compute → feedback during approach → result when grasped). Trade-off: significantly more complex, premature at this stage. The topic-based approach is simpler and decouples the trajectory node from the preshaping node's lifecycle.
