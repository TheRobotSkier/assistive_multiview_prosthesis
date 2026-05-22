# Investigation: Grasp Planner Failing to Grasp Object (Non-Deterministic, "Fully Open" / "No Collision" Results)

## Objective

Diagnose why the interactive simulator's grasp planner (triggered via "Run Planner" button) produces non-deterministic results despite zero covariance, frequently reporting "no collision" or "fully open" (closure_amount=0) grasps, and ultimately failing to physically grasp the object.

## Root Cause Analysis

After thorough investigation of the full pipeline — from the MuJoCo interactive simulator through the depth/pointcloud publisher, the preshaping service bridge, and into the Rust grasp planner — I identified **five primary issues** ranked by severity:

---

### Issue 1 (CRITICAL): Zero Twist + Zero Covariance Collapses the Prediction ROI to a Tiny Point

**Source:** `docker_ws/dev/grasp_preshaping/src/predictor.rs:180-206` and `docker_ws/dev/grasp_preshaping/src/c_api.rs:345-365`

When the hand is stationary (which is the typical case when you click "Run Planner" in the interactive sim), the twist is zero and covariance is zero. The predictor samples 1000 future poses via `sample_future_poses()`, but with zero twist and zero covariance:

- `omega_mean = twist.omega * t = [0,0,0]` for all `t`
- `v_mean = twist.v * t = [0,0,0]` for all `t`
- `noise_omega` and `noise_v` are sampled from `Normal(0, 0)` which is always exactly 0

**Result:** All 1000 sampled poses are **identical** to the current pose. The AABB collapses to a tiny box around the current index fingertip position, inflated by `hand_radius=0.05` and enforced to `min_tsdf_dim=0.1`. This means the TSDF is built from a very small region, and the planner can only evaluate grasps at essentially one hand pose.

This is the **primary cause of non-determinism**: with zero covariance, the results should be deterministic (all samples identical). If you're seeing non-determinism, it's because the twist data arriving from the `read()` callback at `interactive_system_interface.cpp:328-574` varies slightly between planner invocations due to timing — the hand may have micro-velocity from physics noise.

**Why "fully open" or "no collision":** The tiny ROI may not even contain enough object points to build a meaningful TSDF, OR the single hand pose being evaluated simply doesn't produce finger-collision with the object surface.

---

### Issue 2 (CRITICAL): The Planner Evaluates Random Future Poses, Not the Current Hand Pose Itself

**Source:** `docker_ws/dev/grasp_preshaping/src/c_api.rs:354-388`

The planner samples future predicted poses and evaluates grasps at those predicted poses — it never evaluates a grasp at the **current** hand pose. The current pose is only used as the base for prediction, but `sample_future_poses()` always samples at `t ∈ [0, t_max]` where `t=0` is possible but the mean displacement at `t=0` is zero with zero noise.

With zero twist + zero covariance, all samples collapse to the current pose, so this "works" by accident. But if there's any non-zero twist (even noise), the samples spread across a 5-second prediction horizon, most of which will be far from the actual hand position and thus miss the object entirely.

**Impact:** Even when the hand is perfectly positioned over the object, the planner may fail because it's evaluating grasps at random displaced poses rather than at the current hand pose.

---

### Issue 3 (HIGH): Twist Covariance is Velocity-Scaled but Base Values Default to 0.0

**Source:** `docker_ws/dev/mujoco/interactive_simulator/interactive_system_interface.cpp:182-202`

The twist covariance parameters are declared with default values of `0.0`:
- `twist_covariance_linear_base` = 0.0
- `twist_covariance_angular_base` = 0.0
- `twist_covariance_velocity_scale` = 0.0

With `twist_covariance_mode = "velocity_scaled"` (the default), the covariance formula is:
```
cov_lin = linear_base * (1 + velocity_scale * lin_vel_mag)
```

When all three parameters are 0.0, the covariance is always 0.0 regardless of velocity. This means:
- The predictor has **zero exploration noise** — all samples are deterministic
- But the planner was designed to evaluate a **distribution** of future hand poses
- With zero spread, it's essentially evaluating one pose, which may or may not collide

**The non-determinism the user sees** likely comes from the twist values themselves varying slightly between calls (the hand oscillates microscopically in MuJoCo physics), causing the single evaluated pose to shift slightly.

---

### Issue 4 (HIGH): Pointcloud is from a Shadow Model That May Be Stale or Misaligned

**Source:** `docker_ws/dev/mujoco/nodes/mujoco_scene_state_publisher_node.py:490-538`

The scene state publisher runs its own separate MuJoCo model instance ("shadow model"). It receives:
1. Joint states from the interactive simulator
2. Hand/object/camera pose overrides from ROS topics

The depth rendering and pointcloud generation happen on a **timer** (default 5 Hz), and there's a race condition:
- The planner triggers at an arbitrary time
- The latest pointcloud may be up to 200ms stale
- Joint positions and base poses may have changed since the last depth render

Additionally, the shadow model **zeros all finger flexion joints** during depth rendering (`mujoco_scene_state_publisher_node.py:501-503`) to prevent hand self-occlusion. This means the rendered pointcloud shows the object as if the hand were fully open. But the hand pose used by the planner (`/hand_pose`) reflects the actual (possibly partially closed) hand position.

**Impact:** The pointcloud the planner sees may not accurately represent the current scene geometry relative to where the planner thinks the hand is.

---

### Issue 5 (MEDIUM): Closure Commands May Not Produce Physical Grasping Force

**Source:** `docker_ws/dev/grasp_preshaping/src/c_api.rs:163-169` and `docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:166-175`

The planner outputs `closure_amount` (a value between 0 and 1 representing how far the fingers should close). This is sent directly as position commands to the three finger controllers:

```cpp
publish_joint_commands(thumb_closure, index_closure, mrl_closure);
```

For a cylindrical grasp, all three get the same `closure_amount`. For pinch/lateral, `mrl_closure = 0.0`.

Problems:
1. **closure_amount = 0 means "fully open"** — if the planner's best grasp has `found_collision` at sample 0 (i.e., collision at the open position), it reports `closure_amount = 0` and sends 0.0 to all controllers, which opens the hand fully.
2. **The planner scores collision at the TSDF surface, not physical contact** — the TSDF resolution is 5mm and collision tolerance is 5mm, so the "grasp" may close fingers to just barely touch the object surface without generating any gripping force.
3. **No force closure guarantee** — the `force_closure_score` only checks if surface normals are balanced, not whether the grasp actually resists external forces.

---

## Implementation Plan

### Phase 1: Fix the Prediction ROI for Stationary Hand (Addresses Issues 1 & 2)

- [ ] **1.1** Add a fallback in `c_api.rs:compute_from_request()` that, when twist is near-zero and covariance is near-zero, uses a fixed exploration strategy instead of the twist-based predictor. For example, sample hand poses in a small grid around the current pose (e.g., ±2cm translation, ±10° rotation) to evaluate grasps at multiple nearby configurations.
  - **Rationale:** The current predictor is designed for a moving hand predicting future positions. When the hand is stationary (interactive sim), it degenerates.

- [ ] **1.2** Alternatively (or additionally), always include the **current hand pose** as one of the evaluated samples in `score_all_samples()`. This ensures that even if prediction collapses, the planner still tries to grasp at the current position.
  - **Rationale:** The planner should always evaluate the current pose as a candidate.

### Phase 2: Set Meaningful Covariance Defaults for the Interactive Sim (Addresses Issue 3)

- [ ] **2.1** Change the default twist covariance parameters in `interactive_system_interface.cpp:on_activate()` (lines 183-186) to non-zero values that provide reasonable exploration noise:
  - `twist_covariance_linear_base` = 0.001 (1 mm/s variance)
  - `twist_covariance_angular_base` = 0.01 (~6°/s variance)
  - `twist_covariance_velocity_scale` = 0.1
  - **Rationale:** Even with a stationary hand, a small base covariance ensures the predictor explores nearby poses, making the planner more robust.

- [ ] **2.2** Alternatively, make these configurable via the launch file so users can tune them per scenario.
  - **Rationale:** Different use cases (stationary vs. moving hand) need different covariance profiles.

### Phase 3: Improve Planner Robustness for "No Collision" Case (Addresses Issue 5)

- [ ] **3.1** In `c_api.rs:compute_from_request()`, when the best grasp has `closure_amount = 0` (collision at fully open), treat this as a failure case rather than success. The current code at `c_api.rs:391-396` checks `found_collision` but allows `closure_amount = 0`, which produces a "fully open" command.
  - **Rationale:** A grasp that's already colliding at the open position means the hand is intersecting the object before closing — not a useful grasp.

- [ ] **3.2** Add a minimum closure threshold (e.g., `closure_amount >= 0.1`) below which the grasp is rejected as "no useful grasp found."
  - **Rationale:** Prevents sending meaningless zero-closure commands to the hand.

### Phase 4: Reduce Pointcloud Staleness (Addresses Issue 4)

- [ ] **4.1** Increase the depth publishing rate from 5 Hz to at least 15-20 Hz for the interactive sim (the `depth_publish_hz` launch parameter). This reduces staleness from 200ms to ~50-65ms.
  - **Rationale:** The planner should see a reasonably fresh pointcloud.

- [ ] **4.2** Consider triggering a depth render on-demand when the planner is invoked, rather than relying on the timer. This would require adding a service call or topic trigger from the preshaping bridge to the scene state publisher.
  - **Rationale:** Ensures the planner always sees the latest scene geometry.

### Phase 5: Verify Hand-Object Relative Positioning

- [ ] **5.1** Check that the hand's initial position in the interactive scene (`scene_right_dynamic.xml` line 33: sphere at `(-0.1, -0.05, 0.31)`) is actually within reach of the hand's grasp workspace. The hand model's default position should place the fingers around the sphere.
  - **Rationale:** If the hand is too far from the object, no grasp pose will produce collision.

- [ ] **5.2** Add logging in the preshaping bridge node (`preshaping_service_bridge_node.cpp`) to print the hand pose, pointcloud stats (number of points, bounding box), and camera positions when the planner is called. This will help diagnose whether the inputs are reasonable.
  - **Rationale:** Observability is critical for debugging grasp failures.

## Verification Criteria

- [ ] When clicking "Run Planner" with the hand positioned over the sphere, the planner consistently returns a non-zero closure amount
- [ ] The planner result is deterministic when the hand is stationary (same inputs → same outputs)
- [ ] The reported grasp type and closure amount produce a visible finger closure in the simulation
- [ ] The physically simulated hand actually grips the object (doesn't slip through)
- [ ] "No collision" errors only occur when the hand is genuinely too far from the object

## Potential Risks and Mitigations

1. **Risk:** Adding exploration noise may cause the planner to evaluate grasps at unrealistic poses
   **Mitigation:** Start with very small covariance values and increase gradually. The ROI clipping (`max_tsdf_dim=0.3`) already limits the exploration range.

2. **Risk:** Increasing depth publish rate may impact performance
   **Mitigation:** 15-20 Hz is still modest; the depth rendering is offloaded to a separate node with its own MuJoCo instance.

3. **Risk:** Rejecting closure_amount=0 may cause the planner to always fail for certain hand positions
   **Mitigation:** This is actually correct behavior — if the hand can't find a valid grasp, it should report failure rather than sending a zero-closure command that looks like "success."

## Alternative Approaches

1. **Bypass the predictor entirely for the interactive sim:** When the hand is stationary, skip the twist-based prediction and directly evaluate grasps at the current pose with a small perturbation grid. This would be a simpler and more reliable approach for the interactive use case.

2. **Use MuJoCo's built-in collision detection:** Instead of the TSDF-based approach, use MuJoCo's native collision detection to find actual finger-object contacts. This would be more accurate but requires tighter integration with the simulator.

3. **Pre-compute grasp poses:** For known objects (sphere, cylinder), pre-compute optimal grasp poses and closure amounts, then just select the closest one based on current hand position. This trades generality for reliability.
