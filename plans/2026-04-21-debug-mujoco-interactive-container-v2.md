# Debugging the MuJoCo Interactive Container: Grasp Planner Pipeline

## Objective

Diagnose and fix two observed issues in the interactive simulation:
1. **Unclear re-trigger behavior** — pressing "Run Planner" again does not appear to retrigger the planner, or there is no visual/log indication that it did.
2. **Fingers passing through the point cloud** — the preshaping planner outputs a `closure_amount` but the fingers may not be respecting collision geometry.

Additionally, validate that the full pipeline (UI button → ROS service call → Rust planner → joint commands → MuJoCo physics) is functioning correctly end-to-end.

---

## Architecture Overview (for reference)

The interactive container launches the following pipeline:

```
MuJoCo GUI (interactive_simulator.cpp)
  └─ "Run Planner" button → launch_planner()
       └─ sets planner_request_pending_ = true
            └─ consumed in InteractiveSystemInterface::read() (every control cycle)
                 └─ trigger_preshaping_service()
                      └─ calls /grasp_preshaping/compute_grasp (ROS Trigger service)
                           └─ PreshapingServiceBridgeNode (preshaping_service_bridge_node.cpp)
                                ├─ subscribes to /hand_pose (from system interface)
                                ├─ subscribes to /hand_twist (from system interface)
                                ├─ subscribes to /segmented_object_cloud (from depth pipeline)
                                └─ calls Rust grasp_preshaping_compute() FFI
                                     └─ returns closure_amount
                                          └─ publishes to:
                                               ├─ /thumb_pos_ff_controller/commands
                                               ├─ /index_pos_ff_controller/commands
                                               └─ /mrl_pos_ff_controller/commands
```

The depth pipeline runs in parallel:
```
mujoco_scene_state_publisher_node.py (loads shadow model, renders depth, publishes point cloud)
  └─ /mujoco/internal/segmented_object_cloud
       └─ mujoco_depth_publisher_node.py (relay)
            └─ /segmented_object_cloud
```

---

## Issue 1: Re-trigger Behavior

### Root Cause Analysis

The `launch_planner()` method at `interactive_simulator.cpp:638-648` uses an atomic `planner_running_` flag:

```cpp
bool expected = false;
if (!planner_running_.compare_exchange_strong(expected, true)) {
    set_status("Planner already running");
    return;
}
planner_request_pending_.store(true);
set_status("Calling preshaping service");
```

The flag `planner_running_` is only cleared in `report_planner_result()` at `interactive_simulator.cpp:125`:
```cpp
planner_running_.store(false);
```

This is called from `trigger_preshaping_service()` → detached thread → `finish()` lambda in `interactive_system_interface.cpp:737`. The flow is:

1. User clicks "Run Planner" → `planner_running_` = true, `planner_request_pending_` = true
2. `read()` cycle consumes the pending request → calls `trigger_preshaping_service()`
3. Detached thread calls the ROS service → waits up to 15s → calls `report_planner_result()` → sets `planner_running_` = false

**Key concern**: The `preshaping_call_running_` flag in `interactive_system_interface.cpp:723-728` provides a SECOND guard:
```cpp
bool expected = false;
if (!preshaping_call_running_.compare_exchange_strong(expected, true)) {
    InteractiveSimulator::get_instance().report_planner_result(
        false, "Preshaping call already in flight");
    return;
}
```

If `preshaping_call_running_` gets stuck as `true` (e.g., the detached thread crashes or the future never completes), then `planner_running_` will never be cleared, and the button will permanently show "Planner already running".

**Most likely scenario**: The first call works fine (fingers move), but subsequent clicks either:
- (a) Work but produce the same closure amount so nothing visibly changes, OR
- (b) The `preshaping_call_running_` flag is not properly reset because the service call completes but `report_planner_result` is never reached (e.g., exception in the thread), OR
- (c) The `planner_running_` flag is properly reset but the UI status message doesn't update visibly (the status text changes but the user doesn't notice because it says the same thing or the UI doesn't refresh).

### Debugging Steps

- [ ] **1.1. Check the UI status field after first run completes.** After pressing "Run Planner" and the fingers move, look at the "Status" text in the Grasp Planner section. It should say "OK: [grasp_type] grasp, closure=X.XXXX, combined=X.XXXX" or "FAIL: [reason]". If it still says "Calling preshaping service" or "Planner already running", the response path is broken.

- [ ] **1.2. Monitor the ROS service directly.** In a second terminal inside the container, run:
  ```bash
  ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger
  ```
  This bypasses the GUI entirely. Check if the service responds and what message it returns. If this works, the service itself is fine and the issue is in the GUI→service bridge.

- [ ] **1.3. Monitor joint command topics.** Run:
  ```bash
  ros2 topic echo /thumb_pos_ff_controller/commands
  ros2 topic echo /index_pos_ff_controller/commands
  ros2 topic echo /mrl_pos_ff_controller/commands
  ```
  Press "Run Planner" multiple times and observe if new commands are published each time.

- [ ] **1.4. Check if the preshaping service bridge has all required data.** Run:
  ```bash
  ros2 topic echo --once /hand_pose
  ros2 topic echo --once /hand_twist
  ros2 topic echo --once /segmented_object_cloud
  ```
  If any of these is not publishing, the service will fail with "No pose/twist/cloud data received yet".

- [ ] **1.5. Add temporary logging to `trigger_preshaping_service()`.** The detached thread at `interactive_system_interface.cpp:733-762` has no RCLCPP logging for the success path. Add an `RCLCPP_INFO` call after `finish()` to log the result. This would confirm the thread completes. Alternatively, check existing log output for any "Preshaping service unavailable" or "Preshaping timeout" messages.

- [ ] **1.6. Check for race condition in `preshaping_call_running_`.** The flag is set to `true` at `interactive_system_interface.cpp:724` and cleared at `interactive_system_interface.cpp:738` (inside `finish()` lambda). If the service call succeeds but `report_planner_result()` throws (unlikely but possible if `sim_` is null), the flag stays true. Verify that the `finish()` lambda is always called by checking all early-return paths in the thread.

- [ ] **1.7. Verify `consume_planner_request()` is called at the right frequency.** The `read()` method runs at 50 Hz (controller manager update rate). The `consume_planner_request()` at `interactive_system_interface.cpp:309` atomically exchanges the flag. If `read()` is not being called (e.g., controller manager is stuck), the request is never consumed. Check for "Overrun" warnings in the log output — the user's log shows one initial overrun during startup which is expected.

---

## Issue 2: Fingers Passing Through the Point Cloud (CRITICAL FINDING)

### Root Cause Analysis — Confirmed Collision Configuration Issue

**This is the most important finding in this investigation.** The target object has **collision explicitly disabled**.

In `scene_right_dynamic.xml:34-35`:
```xml
<geom name="target_sphere" type="sphere" size="0.03"
  rgba="1 0 0 1" contype="0" conaffinity="0"/>
```

The target sphere has `contype="0"` and `conaffinity="0"`, which means **MuJoCo will never generate contacts between the sphere and any other geom**. This is why fingers pass through it — it is not a physics limitation, it is an intentional (or accidental) configuration choice.

Meanwhile, the hand collision geoms use class `collision_r` which only sets `group="3"` (for visualization filtering) but does **not** explicitly set `contype` or `conaffinity`. By MuJoCo defaults, these geoms will have `contype=1` and `conaffinity=1`. However, since the sphere has `conaffinity=0`, the bitwise AND of `(hand_conaffinity & sphere_contype) | (sphere_conaffinity & hand_contype)` = `(1 & 0) | (0 & 1)` = 0, so no contacts are generated.

The floor geom at `scene_right_dynamic.xml:31-32` has `conaffinity="15"` (binary 1111), and the collision_r class geoms default to `contype=1, conaffinity=1`, so `(1 & 15) | (1 & 1) = 1 | 1 = 1` — the floor collision works fine.

**Why this matters**: The sphere was likely set to non-collidable because it was originally designed as a visual/planning target only, not a physics object. But for the interactive simulation where the user wants to see the fingers close around the object, it needs collision enabled.

### Fix for Issue 2

- [ ] **2.1. Enable collision on the target sphere.** In `scene_right_dynamic.xml:34-35`, change:
  ```xml
  <geom name="target_sphere" type="sphere" size="0.03"
    rgba="1 0 0 1" contype="1" conaffinity="1"/>
  ```
  This will allow MuJoCo to generate contacts between the sphere and the finger collision geoms.

- [ ] **2.2. Do the same for the cylinder scene.** In `scene_right_cylinder.xml`, apply the same change to the target cylinder geom.

- [ ] **2.3. Add mass/inertia to the target body.** Currently `target_sphere_body` has no `<inertial>` tag. When collision is enabled, MuJoCo will auto-compute inertia from the geom, but the body may need mass to respond realistically to contact forces. Consider adding:
  ```xml
  <body name="target_sphere_body" pos="-0.1 -0.0499124 0.3100398">
    <freejoint/>  <!-- only if the object should be movable -->
    <geom name="target_sphere" type="sphere" size="0.03" mass="0.1"
      rgba="1 0 0 1" contype="1" conaffinity="1"/>
  </body>
  ```
  If the object should be fixed in place (simpler), just enabling collision without a freejoint is sufficient — the object will act as an immovable obstacle.

- [ ] **2.4. Verify the fix by rebuilding and testing.** After the XML change, rebuild the container (`docker compose run --build --rm mujoco_interactive`) and press "Run Planner". The fingers should now stop against the sphere instead of passing through.

### Additional Collision Checks

- [ ] **2.5. Verify collision_r class default contype/conaffinity.** The `collision_r` class at `mia_hand_right_grasp_frame.xml:26-28` sets `group="3"` but no `contype`/`conaffinity`. MuJoCo defaults are `contype=1, conaffinity=1`, which should work with the fix above. Verify by checking that finger-finger contacts also work (or are appropriately excluded — the `<contact><exclude>` section at lines 141-147 excludes palm-finger contacts, which is correct).

- [ ] **2.6. Check the MuJoCo timestep and solver settings.** If the timestep is too large or the solver iterations are too few, fast-moving contacts can still be missed (tunneling). The scene doesn't override `<option>`, so MuJoCo defaults apply (timestep=0.002, solver=Newton, iterations=100). This should be sufficient for the slow finger motions involved.

---

## Issue 3: End-to-End Pipeline Validation

### Debugging Steps

- [ ] **3.1. Verify all ROS nodes are running.** Run:
  ```bash
  ros2 node list
  ```
  Expected nodes:
  - `/controller_manager`
  - `/robot_state_publisher`
  - `/mujoco_scene_state_publisher`
  - `/mujoco_depth_publisher`
  - `/mujoco_tf_publisher`
  - `/preshaping_service_bridge`

- [ ] **3.2. Verify topic connectivity.** Run:
  ```bash
  ros2 topic list
  ros2 topic info /segmented_object_cloud
  ros2 topic info /hand_pose
  ros2 topic info /hand_twist
  ```
  Confirm publishers and subscribers are connected.

- [ ] **3.3. Verify the point cloud has valid data.** Run:
  ```bash
  ros2 topic echo --once /segmented_object_cloud --field width
  ```
  If `width` is 0, the point cloud is empty and the planner will fail with "No valid points parsed from PointCloud2" or "No points in ROI".

- [ ] **3.4. Test the full pipeline manually via ROS CLI.** After the simulation is running and the depth publisher has had time to publish at least one point cloud:
  ```bash
  ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger
  ```
  Then check:
  ```bash
  ros2 topic echo --once /thumb_pos_ff_controller/commands
  ```

- [ ] **3.5. Check the planner log directory.** The README mentions logs at `docker_ws/dev/mujoco/log/`. Check for any log files there for detailed planner output.

---

## Potential Risks and Mitigations

1. **Race condition in planner re-trigger**
   - Risk: The `preshaping_call_running_` flag could get permanently stuck if the detached thread encounters an unhandled exception.
   - Mitigation: Wrap the entire thread body in a try-catch that always calls `finish()`. Consider adding a timeout-based safety reset.

2. **Stale point cloud data (shadow model limitation)**
   - Risk: The scene state publisher uses a *separate* MuJoCo model (shadow model without plugin). Joint positions are synced from `/joint_states`, but body poses (hand, object, camera) set via the Scene Control UI are NOT reflected in the shadow model. The point cloud may not match the actual interactive scene.
   - Mitigation: This is a known architectural limitation. The shadow model only knows about joint positions, not body pose overrides. For the interactive container, this means the point cloud is always from the *default* scene pose, not the current UI-adjusted pose. If the user moves the hand via Scene Control, the planner will still see the old pose's point cloud.

3. **Closure amount applies equally to all fingers**
   - Risk: The `publish_joint_commands()` function sends the same `closure_amount` to thumb, index, and mrl controllers. Different grasp types (cylindrical, pinch, lateral) may require different finger positions.
   - Mitigation: This is by design for the current preshaping approach (pre-grasp shape), but may need refinement for realistic grasping.

4. **Container rebuild required for code changes**
   - Risk: Any changes to C++ or Rust code require rebuilding the Docker image.
   - Mitigation: The `docker-compose.yml` already includes `colcon build` in the `mujoco_interactive` command, so restarting the container with `--build` will rebuild. XML scene changes are picked up on rebuild since the XML files are in the mounted volume.

5. **Enabling collision may cause physics instability**
   - Risk: If the object is given a freejoint but insufficient mass/damping, contact forces could launch the object or cause simulation divergence.
   - Mitigation: Start with a fixed object (no freejoint) and only add mobility if needed. Monitor for divergence warnings in the log.

---

## Alternative Approaches

1. **Add a "Reset" button to the GUI** — allows manually clearing `planner_running_` state without restarting the container.

2. **Add ROS logging to the preshaping bridge thread** — currently the success path in `trigger_preshaping_service()` is silent. Adding `RCLCPP_INFO` for each step would make debugging much easier.

3. **Use `ros2 topic hz` to verify publishing rates** — confirms that `/hand_pose`, `/hand_twist`, and `/segmented_object_cloud` are actually being published at expected rates before attempting planner calls.

4. **Replace the atomic flag pattern with a proper state machine** — the current dual-atomic-flag design (`planner_running_` in simulator + `preshaping_call_running_` in system interface) is fragile. A single state machine with explicit transitions would be more robust.

---

## Critique of This Plan

### Strengths
- The collision issue (Issue 2) has been definitively identified — `contype="0" conaffinity="0"` on the target sphere is a smoking gun that explains the finger-through-object behavior completely.
- The re-trigger analysis traces the full flag lifecycle across two files and identifies the most likely failure modes.
- The debugging steps are ordered from least-invasive (observation) to most-invasive (code changes).

### Weaknesses / Gaps
1. **Shadow model point cloud staleness is underemphasized.** If the user has moved the hand via Scene Control, the point cloud still reflects the default hand position. This could cause the planner to compute a grasp for the wrong relative hand-object pose. This should be elevated to a first-class issue.

2. **The plan does not address what "working as it should" looks like for re-triggering.** The user should expect: click → status changes to "Calling preshaping service" → (1-2 seconds) → status changes to "OK: cylindrical grasp, closure=0.XXXX..." → fingers move to new position. Clicking again should repeat this cycle. If the closure amount is the same (because the scene hasn't changed), the fingers won't visibly move on re-trigger, which could look like "nothing happened."

3. **No mention of the `hand_twist` data quality.** The twist is computed from finite differences of the hand pose at ~10 Hz. If the hand is stationary, the twist will be near-zero, which means the planner's motion prediction will generate samples very close to the current pose. This is actually correct behavior for a stationary hand, but worth noting.

4. **The `publish_joint_commands()` sends the same value to all three controllers** — this is called out but not flagged as a potential Issue 3. For a sphere, a cylindrical grasp with equal closure on all fingers is reasonable, but for non-symmetric objects this could produce poor grasps.

5. **Missing: what to do if the planner returns `success=false`.** The current code publishes nothing in the failure case (the `publish_joint_commands` call is only on success). The user would see a "FAIL:" status but no finger movement, which is correct but potentially confusing.

### Recommended Priority Order
1. **Fix the collision configuration** (Issue 2) — this is a one-line XML change with immediate visible impact.
2. **Verify re-trigger works** (Issue 1, steps 1.1-1.3) — quick checks that require no code changes.
3. **Add logging to the service bridge thread** (Issue 1, step 1.5) — this will make all future debugging much easier.
4. **Full pipeline validation** (Issue 3) — confirm everything works end-to-end after the fixes.
