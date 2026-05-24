# Fix: Object Body Name Lookup + Stale Preshaping Data

## Objective

Fix two bugs preventing the grasp runner from working with the cylinder scene:

1. **Object body name hardcoded** — `interactive_simulator.cpp:848` only looks for `target_sphere_body`, but the cylinder scene uses `target_cylinder_body`. This causes `obj_body_id_ = -1`, which means `/mujoco/object_pose` always publishes `(0, 0, 0)`.

2. **Stale preshaping data** — Three of the four preshaping topics use `transient_local` QoS (`preshaping_service_bridge_node.cpp:114,119,123`), so old values from previous runs are delivered to new subscribers immediately on startup. The runner may act on stale data instead of waiting for fresh results from the current service call.

## Implementation Plan

### Bug 1: Object body name fallback (C++ — root cause)

- [ ] **Edit `docker_ws/dev/mujoco/interactive_simulator/interactive_simulator.cpp:848`** — Replace the single hardcoded lookup with a fallback chain that tries multiple body names. The current code is:
  ```cpp
  obj_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "target_sphere_body");
  ```
  Replace with a fallback chain:
  ```cpp
  obj_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "target_sphere_body");
  if (obj_body_id_ < 0) obj_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "target_cylinder_body");
  if (obj_body_id_ < 0) obj_body_id_ = mj_name2id(mj_model_, mjOBJ_BODY, "target_object_body");
  ```
  **Rationale:** `mj_name2id` returns -1 when the name is not found. The existing `read_body_pose` lambda at line 854 already handles `id < 0` by returning early, so the only change needed is the lookup itself. This is a minimal, safe change that doesn't affect existing scenes (sphere scene still works because the first lookup succeeds).

### Bug 2: Stale preshaping data guard (Python — grasp_runner_node.py)

- [ ] **Add a `_preshape_run_id` counter to `grasp_runner_node.py.__init__`** — Add `self._preshape_run_id = 0` alongside the existing preshaping state variables (around line 201). Also add `self._preshape_commit_run_id = 0` to track which run's data was last committed.

- [ ] **Stamp preshaping callbacks with the current run ID** — In each of the four preshaping subscription callbacks (`_on_preshaping_wrist`, `_on_preshaping_closures`, `_on_preshaping_target_pose`, `_on_preshaping_grasp_type`), capture `self._preshape_run_id` at callback invocation time and store it alongside the buffer data. Modify `_try_commit_preshaping` to only commit when all four buffers have the same run ID matching `self._preshape_run_id`.

  **Rationale:** This is cleaner than a timestamp guard because the run ID is monotonically increasing and doesn't suffer from timing edge cases. Stale data from a previous run will have a different (lower) run ID and will be ignored.

- [ ] **Increment `_preshape_run_id` in `_on_trigger`** — After resetting the preshaping buffers (around line 347), increment `self._preshape_run_id += 1`. This ensures any stale data still in-flight from `transient_local` QoS will have an old run ID and be discarded by `_try_commit_preshaping`.

### Alternative approach for Bug 2 (simpler but less robust)

Instead of run IDs, simply use `transient_local` QoS on the runner's subscriptions to match the publisher's QoS, and clear buffers in `_on_trigger` (which already happens). The risk is that `transient_local` messages may arrive *after* the trigger resets the buffers, re-populating them with stale data. The run ID approach is more reliable.

## Verification Criteria

- [ ] **Object pose is correct for cylinder scene** — After rebuild, `ros2 topic echo /mujoco/object_pose --once` should show `(-0.1, -0.05, 0.31)` instead of `(0, 0, 0)` when using the cylinder scene.
- [ ] **No stale preshaping data on startup** — The `=== Preshaping Result Received ===` log should NOT appear on startup before any trigger is called. It should only appear after a trigger that successfully calls the preshaping service.
- [ ] **Grasp runner sequence executes** — After triggering, the hand should visibly move toward the object, the preshaping service should be called, and the full sequence should play through.
- [ ] **Sphere scene still works** — The existing `scene_right_dynamic.xml` (which uses `target_sphere_body`) should still work without regression.

## Potential Risks and Mitigations

1. **C++ recompilation required** — The `interactive_simulator.cpp` change requires `colcon build --packages-select mia_hand_mujoco`. Mitigation: This is the same build step already required for the CMakeLists fix.

2. **`transient_local` QoS mismatch** — The runner subscribes to preshaping topics with default QoS (volatile, depth 10), but three of the four publishers use `transient_local`. ROS 2 CycloneDDS may not deliver `transient_local` messages to `volatile` subscribers at all in some configurations. This could explain why the runner sometimes doesn't receive preshaping results. Mitigation: The runner should use matching `transient_local` QoS for those three subscriptions, OR rely solely on the service call response to know when data is ready. Since the service call is synchronous (returns after publishing), the data should be published before the service responds. The runner already waits for the service future to complete before checking the buffers.

3. **CycloneDDS RTPS payload error** — The `Change payload size of '24' bytes is larger than the history payload size of '11' bytes` error from the user's logs suggests a DDS serialization mismatch for the Trigger service response. This is a known CycloneDDS issue with service types. Mitigation: This doesn't affect functionality (the service still processes), but it prevents the CLI client from receiving the response. The runner handles this internally via `call_async` + future polling, which works regardless.

## Alternative Approaches

1. **Make object body name a ROS parameter** — Instead of a hardcoded fallback chain, read the body name from a parameter. More flexible but requires parameter wiring in the launch file and docker-compose. The fallback chain is simpler and sufficient.

2. **Use a single combined preshaping topic** — Instead of four separate topics, publish a single combined message. This eliminates the stale data problem entirely but requires changing the preshaping bridge node (C++). Not worth the scope for this fix.

3. **QoS matching on runner subscriptions** — Subscribe to the three `transient_local` preshaping topics with matching `transient_local` QoS. This ensures the runner always gets the latest data. Combined with the run ID guard, this is the most robust approach.
