# Preshaping Debug Plan: Fingers Passing Through Object

## Objective

Diagnose and fix the grasp preshaping pipeline where:
1. The planner reports "OK: lateral grasp" or "OK: cylindrical grasp" but fingers pass through the object.
2. The default pose sends 1.0 to all three controllers (fully closed), while a slightly-lowered pose sends ~0.7-0.78.
3. The system does not use the camera pose for inside/outside TSDF calculation.

---

## Root Cause Analysis

After examining the full codebase, I have identified **multiple compounding issues** ranked by severity. Each cites the exact file and line range.

---

### Issue 1 (CRITICAL): TSDF unit mismatch — the entire TSDF grid collapses to a single cell

**Source**: `docker_ws/dev/grasp_preshaping/src/c_api.rs:19-21` and `c_api.rs:338-346`

```rust
const TSDF_RESOLUTION_MM: f32 = 5.0;    // line 19
// ...
let (morton_arr, offsets, start) = morton(&pruned, TSDF_RESOLUTION_MM);  // line 338
let tsdf = get_tsdf(&morton_arr, &offsets, TRUNCATION_CELLS, start, TSDF_RESOLUTION_MM, &[]);  // line 339
```

The point cloud from MuJoCo is in **meters** (sphere center at approximately `(-0.1, -0.05, 0.31)` — see `scene_right_dynamic.xml:33`). The `morton()` function in `pointcloud_helper.rs:293-346` divides point coordinates by `resolution_mm` to get grid indices:

```rust
let gx = ((p.x - min.x) / resolution_mm).floor() ...  // pointcloud_helper.rs:312
```

With `resolution_mm = 5.0` and meter-scale coordinates:
- Point at x = -0.1 → relative to min, then divided by 5.0 → essentially 0
- Point at x = 0.31 → relative to min, then divided by 5.0 → essentially 0

**All points collapse to grid cell 0 or 1.** The TSDF grid is 1-2 cells wide, completely degenerate. The truncation band of 4 cells extends the grid slightly, but the entire scene is within a few cells. The collision detection cannot meaningfully detect fingers approaching the object.

**Evidence from unit tests**: The tests in `pointcloud_helper.rs:552-610` use resolution `1.0` with coordinates like `(10.0, 10.0, 10.0)`, `(20.0, 20.0, 20.0)` — clearly abstract "grid units" where 1 unit = 1 cell. The real system uses meter coordinates with resolution 5.0, which is 5000x too coarse.

**Impact**: This is the primary cause of fingers passing through the object. The TSDF cannot represent the 3cm-radius sphere meaningfully, so the sweep never detects collision, returns `None`, and the planner outputs `closure_amount: 1.0` (fully closed).

### Issue 2 (CRITICAL): No camera pose passed to TSDF — inside/outside detection disabled

**Source**: `docker_ws/dev/grasp_preshaping/src/c_api.rs:345`

```rust
let tsdf = get_tsdf(&morton_arr, &offsets, TRUNCATION_CELLS, start, TSDF_RESOLUTION_MM, &[]);
//                                                                                   ^^ empty cameras
```

The `get_tsdf` function (`pointcloud_helper.rs:348-489`) has camera-based inside/outside sign flipping logic (lines 434-479). When `cameras` is empty, this entire block is skipped. All TSDF distances are positive (outside). Without negative distances, the planner cannot distinguish "finger inside object" from "finger far from object."

Even after fixing Issue 1, without camera data the planner would still lack inside/outside information. The camera pose IS available on `/mujoco/camera_pose` (published by the interactive system interface at `interactive_system_interface.cpp:151,443`), but it is never passed to the planner.

### Issue 3 (HIGH): Closure amount 1.0 on no-collision means "fully close all fingers"

**Source**: `docker_ws/dev/grasp_preshaping/src/planner.rs:275-280`

```rust
match sweep_for_collision(...) {
    None => GraspScoreResult { closure_amount: 1.0, ... },  // line 277
    Some(0) => GraspScoreResult { closure_amount: 0.0, ... },
    Some(coll_sample) => { /* ... proper scoring ... */ }
}
```

When `sweep_for_collision` returns `None` (no collision found during the entire finger-closing sweep), the result is `closure_amount: 1.0`. This means "close all the way." Combined with Issue 1 (degenerate TSDF that never detects collision), the planner **always** returns 1.0 for all grasp types when the hand is near the object.

The bridge node (`preshaping_service_bridge_node.cpp:247`) then publishes this 1.0 to all three finger controllers, driving them fully closed through the object.

### Issue 4 (HIGH): Same closure amount sent to all three finger groups regardless of grasp type

**Source**: `docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:152-159`

```cpp
void publish_joint_commands(double position) {
    command.data = {position};
    thumb_cmd_pub_->publish(command);
    index_cmd_pub_->publish(command);
    mrl_cmd_pub_->publish(command);
}
```

Even when the planner correctly identifies a grasp type (lateral vs. cylindrical vs. pinch), the bridge sends the same scalar to thumb, index, and MRL controllers. Different grasp types require fundamentally different finger configurations:
- **Lateral**: thumb adducted against index side, MRL fingers locked
- **Cylindrical**: all fingers wrap around the object
- **Pinch**: thumb tip meets index tip, MRL locked

The current architecture cannot express these differences.

### Issue 5 (MEDIUM): All grasp type weights are identical

**Source**: `docker_ws/dev/grasp_preshaping/src/planner.rs:35-58`

All three `GraspWeights` constructors return `{1.0, 1.0, 1.0}`. The combined score formula (`planner.rs:15-24`) is:

```rust
(w_probability * prob + w_alignment * alignment + w_force_closure * force_closure) / denom
```

With identical weights, the grasp type selection is purely driven by which raw metric happens to be highest, with no bias toward the appropriate grasp for the object geometry. This explains why the planner may report "lateral" for a sphere (where cylindrical is clearly more appropriate).

### Issue 6 (MEDIUM): Sparse point cloud for small objects

**Source**: `docker_ws/dev/mujoco/nodes/mujoco_scene_state_publisher_node.py:404-418`

The point cloud uses `pointcloud_stride=2` (launch default), skipping every other pixel. For a 3cm-radius sphere viewed from ~46cm away with fovy=58° on a 640×480 sensor, the sphere occupies roughly 40×40 pixels. After stride=2, that's ~20×20 = 400 points, which is reasonable but could be denser.

The depth publish rate is 5 Hz (`depth_publish_hz` default). If the hand moves between cloud capture and planning, the TSDF will be stale.

### Issue 7 (LOW): Zero twist in static sim produces very tight ROI

**Source**: `docker_ws/dev/mujoco/gui_simulator/planner_gui_system_interface.cpp:298-313`

The GUI simulator publishes zero twist with tiny covariance (1e-6). The predictor generates samples clustered tightly around the current pose. The ROI is then inflated to `MIN_TSDF_DIM = 0.1m` minimum. For the default hand position, this should still contain the sphere, but if the hand is moved far from the object, the pruned cloud could be empty.

---

## Implementation Plan

### Phase 1: Fix the critical TSDF unit mismatch

- [ ] **Task 1.1**: Change `TSDF_RESOLUTION_MM` from `5.0` to `0.005` in `c_api.rs:19`. This makes the resolution 5mm expressed in meters, matching the point cloud units. Also update the same constant in `benches/pipeline.rs:11`.

  Rationale: The point cloud from MuJoCo is in meters. The resolution must also be in meters for the grid indices to be meaningful. With resolution 0.005m, a 30cm scene extent produces ~60 grid cells — a reasonable TSDF resolution.

- [ ] **Task 1.2**: Update `COLLISION_TOL_MM` handling in `c_api.rs:348`. Currently:
  ```rust
  let collision_tol = COLLISION_TOL_MM / 1000.0;  // 5.0 / 1000.0 = 0.005
  ```
  The TSDF `get_distance()` returns distances in **cell units** (integers from the BFS), not meters. With resolution 0.005m, 1 cell = 5mm. A collision tolerance of 5mm = 1 cell. Change to:
  ```rust
  let collision_tol = COLLISION_TOL_MM / (TSDF_RESOLUTION_MM * 1000.0);
  ```
  Or more simply, set `collision_tol = 1.0` (1 cell). The same fix is needed in `benches/pipeline.rs:171,202`.

  Rationale: The collision check `tsdf.get_distance() < collision_tol` compares TSDF cell-distance against the tolerance. These must be in the same units.

- [ ] **Task 1.3**: Verify the surface normal computation in `pointcloud_helper.rs:228-255` uses `resolution_mm` as the finite-difference step `h`. After the fix, `h = 0.005 * 0.5 = 0.0025m`, which is correct for meter-scale coordinates. No change needed here.

  Rationale: Confirms the normal computation will still work correctly after the resolution change.

- [ ] **Task 1.4**: Update the unit tests in `pointcloud_helper.rs` that use resolution `1.0` with abstract coordinates (e.g., `(10.0, 10.0, 10.0)`). These tests are unit-agnostic and will continue to pass since they use resolution 1.0 with coordinates in matching units. No changes needed for existing tests, but add a new test that uses meter-scale coordinates with resolution 0.005 to match the production configuration.

  Rationale: Ensures the production configuration is validated by tests.

### Phase 2: Pass camera pose to TSDF for inside/outside detection

- [ ] **Task 2.1**: Add a camera position field to `GraspComputeRequestFFI` in both `ffi_types.hpp` and `c_api.rs`. Add three f64 fields: `cam_px`, `cam_py`, `cam_pz`.

  Rationale: The FFI struct must carry the camera position from the C++ bridge to the Rust planner.

- [ ] **Task 2.2**: In `preshaping_service_bridge_node.cpp`, subscribe to `/mujoco/camera_pose` (or reuse the camera pose if already subscribed). Extract position and populate the new FFI fields before calling `grasp_preshaping_compute`.

  Rationale: The camera pose is published by the interactive system interface at `/mujoco/camera_pose`. The bridge node needs to cache and pass it.

- [ ] **Task 2.3**: In `compute_from_request()` in `c_api.rs`, construct a `Camera` from the request fields and pass `&[camera]` to `get_tsdf()` instead of `&[]`.

  Rationale: Enables the inside/outside sign flipping logic in `get_tsdf()`.

- [ ] **Task 2.4**: Consider whether to require the camera pose (error if missing) or make it optional (skip sign flipping if no camera). Recommend making it optional for backward compatibility but logging a warning.

  Rationale: The planner should still work without camera data, just with reduced accuracy.

### Phase 3: Fix dangerous no-collision default behavior

- [ ] **Task 3.1**: In `planner.rs:275-280`, change the `None` branch to return a low combined score instead of `closure_amount: 1.0`. For example:
  ```rust
  None => GraspScoreResult { closure_amount: 0.0, alignment_score: 0.0, force_closure_score: 0.0 }
  ```
  This means "if no collision is detected, don't close the fingers" — a safe default.

  Rationale: Fully closing when no collision is detected is the most dangerous behavior. The safe default is to not close.

- [ ] **Task 3.2**: Alternatively, distinguish between "no collision because the object is too far" (should return low closure) and "no collision because the sweep completed without hitting anything" (might indicate the object is very small). The current code cannot distinguish these cases, but the `pruned.is_empty()` check in `c_api.rs:334` catches the "no points in ROI" case.

  Rationale: Prevents false negatives where the planner refuses to close even when it should.

### Phase 4: Differentiate grasp type outputs (improvement, not blocking)

- [ ] **Task 4.1**: Add separate closure fields to `GraspComputeResponseFFI` for thumb, index, and MRL finger groups, plus a thumb opposition field. Update both `ffi_types.hpp` and `c_api.rs`.

  Rationale: Different grasp types require different finger configurations.

- [ ] **Task 4.2**: In the planner, compute per-finger-group closure amounts based on which contacts triggered collision for each grasp type. For cylindrical, all groups close together. For lateral, only thumb and index close. For pinch, only thumb and index close partially.

  Rationale: Produces physically correct preshapes for each grasp type.

- [ ] **Task 4.3**: In the bridge node, publish the per-finger-group commands to the appropriate controllers instead of the same value to all three.

  Rationale: The hardware needs different commands for different fingers.

- [ ] **Task 4.4**: Tune `GraspWeights` for each grasp type. For example:
  - Cylindrical: `{ w_probability: 0.5, w_alignment: 0.3, w_force_closure: 1.0 }` — prioritize wrap-around
  - Lateral: `{ w_probability: 0.5, w_alignment: 1.0, w_force_closure: 0.5 }` — prioritize thumb-to-side alignment
  - Pinch: `{ w_probability: 0.5, w_alignment: 1.0, w_force_closure: 0.3 }` — prioritize tip-to-tip alignment

  Rationale: Provides grasp-type-specific scoring bias.

### Phase 5: Improve point cloud quality

- [ ] **Task 5.1**: Set `pointcloud_stride` to 1 in the launch configuration for better TSDF quality with small objects.

  Rationale: More points = better TSDF representation of the object surface.

- [ ] **Task 5.2**: Consider increasing `TRUNCATION_CELLS` from 4 to 6-8 for a wider TSDF band, giving collision detection more margin.

  Rationale: With resolution 0.005m and truncation 4, the band is 20mm. Fingers may approach faster than this band accommodates.

### Phase 6: Verification and testing

- [ ] **Task 6.1**: Add diagnostic logging to `compute_from_request()` that prints: point count (raw and pruned), TSDF grid dimensions, collision_tol value, and the sweep result for each grasp type.

  Rationale: Makes future debugging possible from log files.

- [ ] **Task 6.2**: Write a standalone integration test that:
  1. Creates a point cloud of a sphere at a known meter-scale position
  2. Builds a TSDF with resolution 0.005
  3. Queries distances at the sphere center (should be negative with camera) and at points 30mm away (should be positive)
  4. Runs the full scoring pipeline and verifies the sweep detects collision

  Rationale: End-to-end validation of the fixed pipeline.

- [ ] **Task 6.3**: In the running simulation, verify the fix by:
  1. Echoing `/segmented_object_cloud` and confirming points are in the range -0.2 to 0.5 (meters)
  2. Triggering the planner and checking the log output for TSDF grid dimensions (should be ~40-60 cells per axis)
  3. Verifying the published finger commands are between 0.3 and 0.8 (not 1.0)
  4. Observing in the MuJoCo viewer that fingers stop near the object surface

  Rationale: Confirms the fix works in the actual simulation.

---

## Verification Criteria

1. **TSDF grid dimensions**: After fix, the TSDF should have ~40-80 cells per axis (not 1-2). Log output should confirm this.
2. **Distance query sanity**: `get_distance()` at the sphere center should return ~0.0 (or negative with camera). At 30mm away, should return ~6.0 cells (30mm / 5mm per cell).
3. **Closure amount**: Published finger commands should be 0.3-0.8 range when the hand is near the object, NOT 1.0.
4. **Grasp type**: For a sphere, cylindrical should score highest. For a thin/flat object, lateral or pinch should score highest.
5. **Visual verification**: In the MuJoCo viewer, fingers should stop at or near the object surface without passing through.

---

## Potential Risks and Mitigations

1. **Risk**: Changing `TSDF_RESOLUTION_MM` from 5.0 to 0.005 increases the TSDF grid size dramatically (~1000x per axis), potentially causing memory issues or slowdowns.
   **Mitigation**: The ROI pruning (`predict_roi_with_samples`) limits the point cloud to a ~0.1-0.3m AABB. With 5mm resolution, that's 20-60 cells per axis = 8,000-216,000 cells. The BFS-based TSDF construction handles this efficiently. Profile after the change.

2. **Risk**: Adding camera pose to the FFI struct changes the ABI, requiring coordinated rebuild of both Rust and C++ components.
   **Mitigation**: This is expected. Document the change and rebuild both sides. The `dlopen`-based loading means the Rust library just needs to be rebuilt.

3. **Risk**: Changing the no-collision default from `closure_amount: 1.0` to `0.0` may cause the hand to never close if there's a subtle issue with the collision detection.
   **Mitigation**: This is actually the desired behavior — it's safer to not close than to close through an object. Debug why collision isn't detected rather than reverting this change.

4. **Risk**: The camera-based inside/outside detection may misclassify voxels with a sparse point cloud.
   **Mitigation**: The ray-alignment threshold (0.8) is conservative. With a reasonable camera view, this should work well. Can be tuned later.

---

## Alternative Approaches

1. **Scale point cloud to millimeters instead of fixing resolution**: Multiply all point coordinates by 1000.0 in `pointcloud_view_to_pointcloud()`. This would make the existing `TSDF_RESOLUTION_MM = 5.0` correct. However, the LUT positions are in meters, so collision query positions would also need conversion (multiply by 1000.0 before calling `get_distance()`). This is more error-prone than fixing the resolution constant but avoids changing the constant's semantics.

2. **Replace TSDF with analytical SDF**: For known object shapes (sphere, cylinder), compute the signed distance analytically instead of building a voxel grid. This eliminates unit issues entirely and is more accurate. Requires the planner to know the object type and parameters.

3. **Use MuJoCo collision detection**: Query MuJoCo's built-in collision detection directly instead of reconstructing it from a point cloud. Most accurate approach but requires tighter simulator integration.

---

## Recommended Execution Order

1. **Phase 1** (TSDF unit fix) — the single most impactful change; fixes the core collision detection
2. **Phase 3** (safe no-collision default) — prevents dangerous behavior even if detection is imperfect
3. **Phase 6.3** (verify in running sim) — confirm Phase 1 fix works before proceeding
4. **Phase 2** (camera pose for inside/outside) — improves accuracy
5. **Phase 5** (point cloud quality) — improves reliability
6. **Phase 4** (grasp differentiation) — polish and correctness improvement
7. **Phase 6.1-6.2** (logging and tests) — ongoing throughout
