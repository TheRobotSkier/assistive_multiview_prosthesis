# Upgrade Interactive Simulation for Realistic Grasp Preshaping Testing

## Objective

Transform the interactive simulation from a static hand + fixed camera setup into a dynamic scenario that approximates a real user's grasping behavior:

1. Object is visible from the head-mounted camera
2. User approaches the object with their prosthetic hand
3. Preshaping calculation is triggered
4. The actual close command is sent when the object is within a threshold distance of the calculated grasp pose

Additionally, improve the twist covariance to enable better sampling, and ensure all camera TFs (front depth cam + wrist cam) are published correctly for the multiview-centered system.

---

## Current State Analysis

### What exists today
- **Interactive simulator** (`interactive_simulator.cpp`): Full MuJoCo GUI with scene control, motion control, and grasp planner UI. Hand/object/camera can be repositioned via sliders or ROS topics.
- **System interface** (`interactive_system_interface.cpp`): ros2_control hardware interface that bridges MuJoCo to ROS. Publishes hand pose, twist, IMU data, and calls the preshaping service.
- **Scene XMLs**: `scene_right_dynamic.xml` has the hand at `(-0.1, 0, 0.2)` and the front camera at `(-0.08, -0.46, 0.36)` (fixed in world, NOT head-mounted). The wrist cam is attached to `palm_r` at `(0, 0.095, -0.04)`.
- **Preshaping bridge** (`preshaping_service_bridge_node.cpp`): Subscribes to `/hand_pose`, `/hand_twist`, `/segmented_object_cloud`. Looks up camera TFs via tf2. Calls Rust FFI.
- **Rust predictor** (`predictor.rs`): Uses twist + covariance to sample future hand poses, builds ROI AABB, prunes point cloud, scores grasps. Currently the covariance is set to `1e-6` on the diagonal (near-zero) in the system interface, meaning almost no sampling spread.

### Key problems identified (ranked by severity)

**P1 (Critical): Scene state publisher does not track hand base pose changes**
The `mujoco_scene_state_publisher_node.py` maintains its own separate MuJoCo model instance. It only updates joint angles from `/joint_states` (`_apply_joint_positions_unlocked` at line 384). When the hand base body (`palm_r`) is moved via Scene Control or Motion Control in the interactive simulator, the publisher's model still has the hand at its XML-default position. This means:
- Wrist camera TF is wrong (it's a child of `palm_r`)
- Depth rendering is from the wrong viewpoint (the front camera is also at its default position)
- Point cloud is wrong
- The preshaping bridge gets incorrect camera positions from TF

**P2 (High): Camera is NOT head-mounted**
The front depth camera (`depth_cam_body`) is fixed in world frame at a position/angle that doesn't resemble a head-mounted viewpoint. A real user would have a head-mounted camera that sees the object as they approach.

**P3 (High): Twist covariance is too small for useful sampling**
`interactive_system_interface.cpp:428-436` sets all covariance diagonal entries to `1e-6`. The predictor's `sample_twist()` at `predictor.rs:139-144` scales noise by `sqrt(covariance_diagonal) * sqrt(t)`. With `1e-6`, the noise is negligible over any reasonable time horizon, producing a tiny ROI and no exploration of alternative grasp configurations.

**P4 (Medium): No approach-and-grasp workflow**
Currently the user manually clicks "Run Planner" at an arbitrary moment. There is no automated flow: detect object visible, approach, trigger preshaping, close when within threshold.

---

## Implementation Plan

### Phase 1: Fix Scene State Publisher to Track Base Pose (P1 — Unblocks Everything)

- [ ] **1.1 Subscribe to hand/object/camera pose topics in the scene state publisher**  
  In `mujoco_scene_state_publisher_node.py`, add subscriptions to `/mujoco/hand_pose`, `/mujoco/object_pose`, and `/mujoco/camera_pose` (already published at ~10 Hz by the system interface). Cache the latest values.  
  *Rationale*: The publisher needs to know where the base bodies are in order to render depth and publish correct TFs.

- [ ] **1.2 Apply base body pose overrides before each render/TF update**  
  In `_apply_joint_positions_unlocked()` (or a new method called alongside it), write the cached hand/object/camera poses into the publisher's MuJoCo model's `body_pos` and `body_quat` arrays for the corresponding body IDs (`palm_r`, `target_sphere_body`/`target_cylinder_body`, `depth_cam_body`). This requires looking up body IDs by name at initialization.  
  *Rationale*: Without this, the publisher's model is stuck at the XML-default positions and all downstream data (TF, depth, point cloud) is wrong when any body is moved.

- [ ] **1.3 Add a parameter to enable/disable base pose tracking**  
  Add `track_base_poses` parameter (default `true` for dynamic scenes). When disabled, the publisher behaves as before (only joint states).  
  *Rationale*: Backwards compatibility for static scenes where bodies don't move.

### Phase 2: Fix Twist Covariance for Better Sampling (P3 — Small Change, Big Impact)

- [ ] **2.1 Replace hardcoded `1e-6` covariance with configurable parameters**  
  In `interactive_system_interface.cpp:428-436`, replace the fixed values with parameters read at `on_activate` time:
  - `twist_covariance_linear` (default: `0.01`) — linear velocity variance (m/s)^2
  - `twist_covariance_angular` (default: `0.005`) — angular velocity variance (rad/s)^2
  
  These produce standard deviations of ~10 cm/s and ~4 deg/s respectively, which give meaningful sampling spread over the 5-second prediction horizon.  
  *Rationale*: The current `1e-6` is effectively zero. The predictor's sampling at `predictor.rs:139-144` uses `sqrt(cov) * sqrt(t)`, so `0.01` gives `0.1 * sqrt(5) ≈ 0.22 m` spread at the horizon—reasonable for an approaching hand.

- [ ] **2.2 Implement velocity-scaled covariance mode**  
  When the hand is moving, scale the covariance proportionally to the measured velocity magnitude. A simple model:
  - `cov_linear = base_cov_linear * (1.0 + k_scale * |v_linear|)`
  - `cov_angular = base_cov_angular * (1.0 + k_scale * |v_angular|)`
  - When stationary, use the base values (still non-zero, representing estimation uncertainty)
  
  The twist computation already exists in `interactive_system_interface.cpp:391-417` (it computes `world_lin_vel` and angular velocity from finite differences). Use those computed velocities for scaling.  
  *Rationale*: The user requested deriving covariance from "actual pose estimation of the hand" where possible. Velocity-scaled covariance is the most realistic approach without a full pose estimation pipeline. Faster motion = more uncertainty = wider sampling.

- [ ] **2.3 Add a `twist_covariance_mode` parameter**  
  Options: `fixed` (constant values), `velocity_scaled` (proportional to speed). Default: `velocity_scaled`.  
  *Rationale*: Enables experimentation without recompilation.

### Phase 3: Make the Camera Head-Mounted (P2 — Scene Changes)

- [ ] **3.1 Reposition the front camera to a head-mounted default**  
  In `scene_right_dynamic.xml` (or a new variant), change `depth_cam_body` position from `pos="-0.08 -0.46 0.36" euler="-1.67079632679 0 0"` to something like `pos="0.05 -0.20 0.55" euler="-2.0 0 0"` — above and slightly behind the hand, looking down toward the workspace. The exact values need empirical tuning to ensure the object is visible in the depth image.  
  *Rationale*: The current camera is far behind and below the hand at an extreme angle. A head-mounted camera should be above the hand, looking down at the workspace where the object sits.

- [ ] **3.2 Keep the front camera as a free movable body**  
  `depth_cam_body` is already a free body in the worldbody of the dynamic scene. The interactive simulator already supports moving it via Scene Control / Motion Control / ROS topics. No code changes needed—just the default position.  
  *Rationale*: A real head-mounted camera moves independently of the hand. Keeping it as a free body preserves this flexibility.

- [ ] **3.3 Verify the front camera can see the object at the new position**  
  After repositioning, launch the interactive sim and switch the viewport to the `front_depth_cam` camera. Verify the sphere/cylinder is visible in the depth rendering. Adjust the camera position/orientation until the object is well-framed.  
  *Rationale*: If the object isn't visible, the segmented point cloud will be empty and preshaping will fail.

### Phase 4: Verify All Camera TFs Are Correct (Depends on Phase 1)

- [ ] **4.1 Verify front camera TF frame name consistency**  
  The scene state publisher uses `camera_frame_id` parameter (default `mujoco_front_depth_cam`) for the primary camera (`mujoco_scene_state_publisher_node.py:308`). The preshaping bridge's `camera_frames` parameter lists `mujoco_front_depth_cam` as the first entry (`preshaping_service_bridge_node.cpp:53`). These should match.  
  *Verification*: Run the sim, `ros2 topic echo /tf --once` and grep for `mujoco_front_depth_cam`.

- [ ] **4.2 Verify wrist camera TF frame name consistency**  
  The scene state publisher names non-primary cameras as `mujoco_camera_{sanitize(name)}` (`mujoco_scene_state_publisher_node.py:310`). For camera name `wrist_cam`, this becomes `mujoco_camera_wrist_cam`. The preshaping bridge lists `mujoco_camera_wrist_cam` as the second entry (`preshaping_service_bridge_node.cpp:54`). These should match.  
  *Verification*: `ros2 topic echo /tf --once` and grep for `mujoco_camera_wrist_cam`.

- [ ] **4.3 Verify both camera TFs update when the hand moves**  
  After Phase 1 is complete, move the hand via Scene Control and verify that:
  - `mujoco_camera_wrist_cam` TF updates (wrist cam is child of `palm_r`, should track hand)
  - `mujoco_front_depth_cam` TF updates (front cam is a free body, should track its own position)
  - The preshaping bridge logs show 2 cameras resolved (not the fallback estimate from `preshaping_service_bridge_node.cpp:265-285`)

- [ ] **4.4 (Optional) Add wrist-camera depth rendering for full multiview**  
  Currently only the front camera renders depth. For full multiview testing, extend the scene state publisher to support multiple cameras with independent rendering pipelines. This is a larger change—defer unless needed immediately.

### Phase 5: Implement the Approach-and-Grasp Workflow (P4 — Builds on All Previous)

- [ ] **5.1 Create a new approach controller node**  
  Create `docker_ws/dev/mujoco/nodes/approach_controller_node.py` (or C++ if preferred) that orchestrates the approach sequence. This node:
  - Subscribes to `/mujoco/hand_pose`, `/mujoco/object_pose`, `/mujoco/sim_time`
  - Publishes to `/mujoco/move_hand` for smooth hand motion
  - Has a service `/mujoco/start_approach_grasp` (`std_srvs/Trigger`) to initiate the sequence
  - Calls `/grasp_preshaping/compute_grasp` when within trigger distance
  - Publishes to `/thumb_pos_ff_controller/commands`, etc. when within threshold distance
  
  *Rationale*: Keeping this as a separate node (rather than modifying the system interface) maintains clean separation and makes the approach logic independently testable.

- [ ] **5.2 Implement approach trajectory computation**  
  When triggered:
  1. Read current hand pose and object pose
  2. Compute approach direction: `obj_pos - hand_pos` (normalized)
  3. Compute pre-grasp position: `obj_pos - standoff_distance * approach_direction`
  4. Send hand to pre-grasp position via `/mujoco/move_hand` with appropriate duration
  5. Then continue moving toward the object at approach speed
  
  *Rationale*: Mimics how a real user approaches: move to a pre-grasp position first, then close in.

- [ ] **5.3 Implement preshaping trigger during approach**  
  Monitor hand-to-object distance. When it falls below `preshaping_trigger_distance` (default: 0.12 m), call `/grasp_preshaping/compute_grasp`. Store the resulting joint commands.  
  *Rationale*: Preshaping should happen while the hand is still approaching, not after it arrives. The twist is meaningful during motion.

- [ ] **5.4 Implement threshold-based grasp execution**  
  After preshaping returns, continue monitoring distance. When it falls below `grasp_threshold_distance` (default: 0.04 m), send the stored joint commands to the position controllers.  
  *Rationale*: The close command should only fire when the hand is actually close enough to grasp the object.

- [ ] **5.5 Add GUI integration for the approach sequence**  
  Add an "Approach & Grasp" button to the interactive simulator's planner panel. When clicked, it publishes to a ROS topic that the approach controller listens to. Show status: "Approaching...", "Preshaping triggered", "Waiting for threshold...", "Grasp executed".  
  *Rationale*: Makes the workflow accessible from the GUI without needing separate ROS commands.

- [ ] **5.6 Expose approach parameters as ROS parameters**  
  On the approach controller node:
  - `standoff_distance` (default: 0.15 m)
  - `preshaping_trigger_distance` (default: 0.12 m)
  - `grasp_threshold_distance` (default: 0.04 m)
  - `approach_speed` (default: 0.05 m/s)
  
  *Rationale*: Tunable without recompilation.

### Phase 6: Scene Configuration for Approach Testing

- [ ] **6.1 Create `scene_right_approach.xml`**  
  Based on `scene_right_dynamic.xml`, but with:
  - Hand starting position offset from the object by ~25 cm (e.g., hand at `(-0.1, 0.15, 0.2)`, object at `(-0.1, -0.05, 0.31)`)
  - Front camera at head-mounted position (from Phase 3)
  - Table/surface geometry if desired (currently just a floor plane)
  
  *Rationale*: Provides a ready-made scenario for testing the approach workflow.

- [ ] **6.2 Add `approach` as a scene choice in the launch file**  
  In `mia_hand_system_interface_launch.py`, add `'approach': 'scene_right_approach.xml'` to the `scene_file_map` and add `'approach'` to the choices list.

- [ ] **6.3 Update docker-compose to support the approach scene**  
  Add `MUJOCO_SCENE` environment variable support or document how to launch with the approach scene.

---

## Verification Criteria

- [ ] **V1 — Base pose tracking**: Move the hand via Scene Control. Verify that the wrist camera TF (`mujoco_camera_wrist_cam`) updates to reflect the new hand position. Verify the depth image shows the scene from the correct viewpoint.
- [ ] **V2 — Twist covariance**: With `velocity_scaled` mode, a moving hand should produce a larger ROI in the predictor than a stationary hand. The preshaping bridge log should show non-trivial sampling (not just a single point).
- [ ] **V3 — Camera TFs**: Both `mujoco_front_depth_cam` and `mujoco_camera_wrist_cam` are published and contain correct positions. The preshaping bridge logs show 2 cameras resolved.
- [ ] **V4 — Approach workflow**: Trigger the approach sequence. The hand moves toward the object, preshaping is triggered at the correct distance, and the grasp executes when the threshold is reached.
- [ ] **V5 — Backward compatibility**: The existing "Run Planner" button still works independently. Static scenes still function correctly.

---

## Potential Risks and Mitigations

1. **Scene state publisher base pose synchronization lag**  
   The system interface publishes poses at ~10 Hz, and the scene state publisher renders at ~5 Hz. There may be a 1-2 frame lag between moving the hand and the depth/TF updating.  
   *Mitigation*: This lag is acceptable for testing. If it causes issues, increase the pose publish rate or use a shared memory approach.

2. **Front camera visibility during approach**  
   The head-mounted camera must see the object throughout the approach trajectory. If the camera is too close to the hand or at the wrong angle, the object may leave the FOV.  
   *Mitigation*: Start with a conservative camera position (high up, looking down). Add a visibility check in the approach logic that verifies the point cloud is non-empty before triggering preshaping.

3. **Twist covariance tuning sensitivity**  
   Too-large covariance → enormous ROI with irrelevant geometry → degraded grasp quality. Too-small → no exploration.  
   *Mitigation*: Start with values from `TwistCovariance::dummy()` (`0.01` angular, `0.005` linear) as baseline. The parameter-based approach allows runtime tuning.

4. **MuJoCo body_pos vs qpos confusion**  
   The hand's base position is stored in `body_pos` (model-level, not data-level in MuJoCo). The interactive simulator modifies `m->body_pos` directly (not `d->qpos`). The scene state publisher needs to do the same on its own model instance.  
   *Mitigation*: The publisher already calls `mj_forward()` after updating joint positions. Writing `body_pos` before `mj_forward()` will propagate correctly to `xpos`, `xmat`, etc.

5. **Approach trajectory collision with floor/table**  
   The hand might collide with the floor plane during approach if the trajectory is not planned carefully.  
   *Mitigation*: The approach controller computes a straight-line trajectory in 3D. Ensure the start and end positions are above the floor. The floor collision in MuJoCo will prevent penetration but may cause unrealistic behavior.

---

## Alternative Approaches

1. **Modify system interface directly instead of separate approach node**  
   Embed the approach logic in `interactive_system_interface.cpp` alongside the existing planner trigger.  
   *Trade-off*: Fewer nodes to manage, but makes the system interface more complex. The separate node approach is cleaner for testing and iteration.

2. **Use MuJoCo mocap bodies for approach trajectory**  
   Use MuJoCo's built-in mocap tracking to drive the hand along a trajectory with physics-correct dynamics.  
   *Trade-off*: More realistic physics but requires significant XML and code changes. The current smoothstep interpolation via `request_hand_move` is sufficient for preshaping testing.

3. **Fixed elevated camera instead of head-mounted**  
   Place the camera at a fixed overhead position that always sees the workspace.  
   *Trade-off*: Simpler but less realistic. Since the project is multiview-centered, realistic camera placement matters for valid testing. The free-body approach (Phase 3) is only slightly more complex and much more realistic.

---

## Recommended Implementation Order

1. **Phase 1** (base pose tracking) — Critical blocker, unblocks TF/depth correctness
2. **Phase 2** (twist covariance) — Small change, immediate improvement in preshaping quality
3. **Phase 3** (head-mounted camera) — Scene XML changes only, enables realistic testing
4. **Phase 4** (TF verification) — Verify the above work correctly together
5. **Phase 6** (approach scene) — New scene variant
6. **Phase 5** (approach workflow) — Most complex, builds on all previous phases
