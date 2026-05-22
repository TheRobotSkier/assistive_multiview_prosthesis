# Diagnose "Run Planner" Button Only Works First Time

## Objective

Determine why the grasp planner returns valid grasps on the first button press but returns zero-closure (open hand) results on all subsequent presses, even after the hand has reopened.

## Current Understanding

### Confirmed Behavior
- **Click 1**: Planner finds collisions, returns non-zero closure → hand closes
- **Click 2**: Planner returns zero closure → hand opens
- **Click 3+**: Planner continues returning zero closure → hand stays open
- **Moving the object**: Fixes the issue temporarily (next click works)

### Architecture Overview
1. `PlannerGuiSimulator` — MuJoCo GUI sim with hand, user clicks "Run Planner"
2. `PlannerGuiSystemInterface` — ros2_control hardware plugin, publishes `/hand_pose`, `/mujoco/hand_pose`, `/hand_twist`
3. `MujocoSceneStatePublisher` — Shadow MuJoCo model, renders depth, filters by target geom, publishes `/segmented_object_cloud`
4. `PreshapingServiceBridgeNode` — Subscribes to `/hand_pose`, `/hand_twist`, `/segmented_object_cloud`, calls Rust planner via FFI
5. Rust planner — Builds TSDF from point cloud, sweeps finger LUT for collisions

### Key Code Paths

**Button press → service call:**
- `planner_gui_simulator.cpp:740-743` → `launch_planner()` sets `planner_request_pending_`
- `planner_gui_system_interface.cpp:318-321` → `consume_planner_request()` → `trigger_preshaping_service()`
- `preshaping_service_bridge_node.cpp:89-95` → service callback → `try_handle_direct_request()`

**Planner result → joint commands:**
- `c_api.rs:456-474` — When `compute_from_request` returns `Ok(output)`, `success = 1` ALWAYS (even if `found_collision = false`)
- `preshaping_service_bridge_node.cpp:312-321` — If `success != 0`, publishes closure values (which are 0.0 when no collision found)

**Point cloud pipeline:**
- `mujoco_scene_state_publisher_node.py:469-498` — `on_depth_timer()` applies joint positions + base poses → `mj_forward()` → render depth → filter by target geom
- `mujoco_scene_state_publisher_node.py:586-619` — `filter_points_by_target_geom()` uses `mj_ray()` to check if each depth pixel ray hits the target geom

## Root Cause Analysis

### Primary Hypothesis: Point Cloud Degradation After Hand Closure

**Mechanism:**
1. Click 1 closes the hand → joint commands flow through ros2_control → MuJoCo GUI sim → `/joint_states`
2. Shadow model receives closed-hand joints → renders depth with closed hand geoms
3. `mj_ray()` in `filter_points_by_target_geom()` finds hand geoms occluding the target → most/all target points filtered out
4. Published `/segmented_object_cloud` is empty or very sparse
5. Subsequent planner calls receive degraded cloud → TSDF has no surface near finger contact points → no collisions found

**Why click 3+ doesn't recover (user's challenge):**

This is the key question. After click 2 publishes zero closure, the hand SHOULD reopen, and the point cloud SHOULD recover. Possible reasons it doesn't:

#### Sub-hypothesis A: The planner returns `success=1` with zero closure even when the cloud is valid

Looking at `c_api.rs:389`:
```rust
let best = select_best_grasp(&scored).ok_or("No valid grasps found")?;
```

`select_best_grasp` uses `max_by` on `combined` scores. When ALL samples have `found_collision = false`, ALL have `combined = NEG_INFINITY`. `max_by` returns the first element (they're all equal). So `select_best_grasp` returns `Some(...)`, and the result has `found_collision = false`, `closure_amount = 0.0`.

This means the planner NEVER returns "no valid grasps found" — it always returns a "successful" result with zero closure when no collisions are detected. The hand opens, but the STATUS shows "OK" (not "FAIL"), which might mislead the user into thinking the planner ran correctly.

**If the point cloud recovers on click 3 but the planner still finds no collisions, the issue might be in the TSDF/collision detection, not the point cloud.**

#### Sub-hypothesis B: Point cloud partially recovers but is insufficient

After the hand reopens, the shadow model should produce a valid point cloud. But:
- The TSDF resolution is 5mm with 4-cell truncation (20mm)
- The collision tolerance is 5mm
- If the point cloud has gaps or reduced density, the TSDF might not have sufficient surface representation near the finger contact points

#### Sub-hypothesis C: Timing — depth timer hasn't updated the cloud yet

The depth timer runs at 5 Hz (200ms intervals). If the user clicks within 200ms of the hand reopening, the cloud might still be from the closed-hand state. But the user reports this persists even after waiting, ruling this out for the general case.

### Secondary Hypothesis: `select_best_grasp` Logic Bug

The function `select_best_grasp` at `c_api.rs:226-232`:
```rust
fn select_best_grasp(scored: &[ScoredGrasp]) -> Option<&ScoredGrasp> {
    scored.iter().max_by(|a, b| {
        a.combined.partial_cmp(&b.combined).unwrap_or(std::cmp::Ordering::Equal)
    })
}
```

When `a.combined = NEG_INFINITY` and `b.combined = NEG_INFINITY`, `partial_cmp` returns `Some(Equal)`. So it returns the first element. This is technically correct (all are equally bad), but the result is treated as a "success" by the bridge node.

**This is a real bug**: The planner should return an error when NO collisions are found, not a "successful" zero-closure result.

## Investigation Plan

- [ ] **Task 1. Add diagnostic logging to `MujocoSceneStatePublisher::on_depth_timer()`** — Log the number of points before and after `filter_points_by_target_geom()` to confirm whether the point cloud is being depleted. Add at `mujoco_scene_state_publisher_node.py:489-491`:
  - Log `points_cam_ros.shape[0]` before filtering
  - Log filtered count after filtering
  - This will definitively show whether occlusion is the cause

- [ ] **Task 2. Add diagnostic logging to the Rust planner** — In `c_api.rs:compute_from_request()`, log:
  - Number of points in the raw cloud
  - Number of points after `prune()`
  - TSDF grid dimensions
  - Number of `ScoredGrasp` results with `found_collision = true` vs `false`
  - Best grasp's `found_collision` and `closure_amount`

- [ ] **Task 3. Verify point cloud recovery timing** — Add timestamp logging to both the scene state publisher's depth timer and the preshaping bridge's service callback to confirm the cloud is fresh when the planner runs.

- [ ] **Task 4. Test with `pc_mode=full`** — Temporarily disable target geom filtering by setting `pc_mode` to `full` to see if the planner works on subsequent clicks when the full scene point cloud is used. This would confirm the filtering is the issue.

## Recommended Fix (Two-Part)

### Part A: Fix the `success` semantics bug (definite bug)

In `c_api.rs:compute_from_request()`, after `select_best_grasp`, check if the best grasp actually found a collision:

At `c_api.rs:389-404`, change:
- If `best.result.found_collision == false`, return `Err("No collision found for any grasp type")` instead of `Ok(ComputeOutput { ... })`

This ensures the bridge node gets `success = 0` when no collision is found, and does NOT publish joint commands (the hand stays in its current position instead of opening).

### Part B: Fix the point cloud occlusion issue (root cause)

In `mujoco_scene_state_publisher_node.py:on_depth_timer()`, **reset the finger flexion joints to 0.0 (open) in the shadow model before rendering depth**:

At `mujoco_scene_state_publisher_node.py:469-498`, after `_apply_joint_positions_unlocked()` and `_apply_base_poses_unlocked()` (lines 473-474), but before `mj_forward()` (line 475):
1. Save the current flexion joint qpos values
2. Set flexion joints (j_thumb_fle, j_index_fle, j_mrl_fle, j_ring_fle, j_little_fle) to 0.0
3. Call `mj_forward()` and render depth
4. Restore the saved flexion values
5. Call `mj_forward()` again for correct body pose publishing

This ensures the depth rendering always uses an open hand, preventing self-occlusion of the target object.

## Verification Criteria

- [Criterion 1] Clicking "Run Planner" multiple times in succession produces valid grasps (non-zero closure) each time
- [Criterion 2] The preshaping bridge returns `success=false` when no collision is found (instead of `success=true` with zero closure)
- [Criterion 3] The shadow model's point cloud contains a consistent number of target object points regardless of hand closure state
- [Criterion 4] Diagnostic logs show point cloud counts and collision detection results for debugging

## Potential Risks and Mitigations

1. **Risk: Zeroing finger joints in shadow model affects body pose accuracy**
   Mitigation: Save and restore joint values; only zero for depth rendering, restore for body pose publishing

2. **Risk: Changing `success` semantics breaks downstream consumers**
   Mitigation: The bridge node already handles `success=false` correctly (returns without publishing commands). The GUI status will show "FAIL:" instead of "OK:", which is more informative.

3. **Risk: Part B fix adds latency to depth timer (double `mj_forward()` call)**
   Mitigation: `mj_forward()` for this small model is sub-millisecond. The depth timer runs at only 5 Hz.

## Alternative Approaches

1. **Alternative A: Use `geomgroup` to exclude hand geoms from `mj_ray()`** — Instead of zeroing joints, set `geomgroup` to exclude hand collision geoms (group 3) during ray casting. This is simpler but doesn't address visual mesh occlusion.

2. **Alternative B: Disable joint position application entirely for depth rendering** — Don't apply any joint positions before rendering; always use the default (open) pose. Simpler but loses the ability to see finger positions in the depth image.

3. **Alternative C: Only fix Part A (success semantics)** — If the point cloud recovers naturally but the planner returns zero closure as "success", fixing just the success check would prevent the hand from opening on failed plans. The user would need to move the hand away and back to get a fresh grasp. This is a minimal fix but doesn't address the root cause.
