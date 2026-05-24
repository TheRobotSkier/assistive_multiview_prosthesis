# Test 1: Software Verification — Implementation Plan

## Objective

Design and execute an offline software verification test that validates the core grasp planning pipeline (TSDF construction, SMC optimization, grasp scoring) using synthetic point cloud data. The test must produce quantitative evidence for requirements 1.6 (autonomous grasp computation), 2.4 (pipeline latency ≤ 400 ms), and 2.7 (multi-view intent precision Δ > 0%, IDE ≥ 10%).

---

## 1. Codebase Assessment

### Key Components Identified

| Component | Location | Role |
|-----------|----------|------|
| Rust grasp planner | `src/grasp_preshaping/src/c_api.rs` | Core `compute_from_request()` — ROI prediction, TSDF, SMC loop, scoring |
| SMC predictor | `src/grasp_preshaping/src/predictor.rs` | Particle sampling, elite selection, resampling |
| Grasp scoring | `src/grasp_preshaping/src/planner.rs` | `score_cylindrical/pinch/lateral`, `GraspScoreResult` |
| Superquadric backside | `src/grasp_preshaping/src/superquadric.rs` | Shape completion for occluded regions |
| Debug export | `src/grasp_preshaping/src/debug_export.rs` | `.npz` dump of TSDF, point cloud, scored grasps |
| C++ bridge node | `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | ROS service wrapping Rust `.so` via FFI |
| Runtime config | `config/grasp_preshaping.yaml` | All tunable parameters (TSDF resolution, SMC iterations, etc.) |
| Pipeline manager | `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py` | State machine (IDLE→SEGMENTING→PLANNING→APPROACHING→GRASPING) |
| Existing integration test | `scripts/test_twist_propagation_integration.py` | Pattern for building mock PointCloud2/PoseStamped messages |

### Critical Observations

1. **The Rust `.so` is self-contained.** The `grasp_preshaping_compute()` FFI function takes a point cloud, pose, twist, and camera positions — it does not need ROS at all. This means the latency test can bypass the entire ROS stack and call the `.so` directly via Python `ctypes`, eliminating network/middleware jitter from the measurement.

2. **Debug dumps already exist.** Setting `debug_visualization: true` in `config/grasp_preshaping.yaml` causes every pipeline invocation to write a `.npz` file with the full TSDF, scored grasps, and camera positions. The existing `visualize_grasp_debug.py` script already parses these. This is the natural data format for the test results.

3. **The FFI response struct already includes `pipeline_time_ms`.** See `GraspComputeResponseFFI` at `src/grasp_preshaping/src/c_api.rs:96`. The Rust code self-times with `std::time::Instant::now()` at `c_api.rs:359` and reports the elapsed time. This is the authoritative latency measurement.

4. **Camera positions are passed per-request.** The `GraspComputeRequestFFI` struct accepts up to 4 camera positions (`c_api.rs:64-68`). The multi-view vs. single-view comparison is therefore trivially controllable at the FFI level — just pass 1 camera for single-view and 2 for multi-view.

5. **The TSDF uses camera positions for ray casting.** Camera positions determine which voxels are "seen" and which are occluded, directly affecting the backside estimation via superquadrics. This is the mechanism that makes multi-view superior: more cameras → more seen voxels → better shape completion → better grasp scoring.

---

## 2. Strategic Approach: Bypass ROS, Test the `.so` Directly

### Why This Is the Right Balance

| Approach | Realism | Ease | Verdict |
|----------|---------|------|---------|
| Full ROS launch with mock nodes | Highest (tests middleware) | Complex setup, variable timing | Overkill for software verification |
| **Python script calling `.so` via ctypes** | **High (tests actual algorithms)** | **Minimal dependencies** | **Recommended** |
| Unit tests in Rust | Lowest (tests components in isolation) | Easy but misses integration | Supplementary |

The ctypes approach is ideal because:
- It exercises the **exact same compiled Rust code** that runs on the robot
- It eliminates ROS communication overhead from latency measurements
- It gives deterministic, reproducible inputs
- It requires only Python + numpy + the pre-built `.so`

---

## 3. Synthetic Dataset Design

### 3.1 Object Models (10 objects as specified)

Generate point clouds programmatically for 10 canonical objects. Each is defined by a mesh or parametric surface, sampled from the surface to produce a dense point cloud (~5,000–10,000 points per object).

| # | Object | Geometry | Rationale | Expected Grasp |
|---|--------|----------|-----------|----------------|
| 1 | Cylinder (upright) | r=3cm, h=12cm | Standard cylindrical grasp target | Cylindrical |
| 2 | Cylinder (tilted 30°) | Same, rotated | Tests orientation robustness | Cylindrical |
| 3 | Sphere | r=4cm | Symmetric, tests isotropic scoring | Cylindrical |
| 4 | Box (large) | 8×8×4 cm | Flat faces, broad contact | Cylindrical |
| 5 | Box (small) | 3×3×3 cm | Small object, pinch target | Pinch |
| 6 | Thin plate | 10×6×0.5 cm | Flat, requires lateral | Lateral |
| 7 | Bottle | r=3.5cm, h=18cm, tapered | Real-world object, partially occluded | Cylindrical |
| 8 | Mug (with handle) | Cylinder + handle points | Complex topology | Cylindrical (handle) or Pinch |
| 9 | Key | 5×2×0.3 cm | Very thin, lateral target | Lateral |
| 10 | Ball (small) | r=1.5cm | Small sphere, pinch | Pinch |

### 3.2 Viewpoint Simulation

For each object, define two camera positions that mimic the real system geometry:

- **Head-mounted camera** (camera_front): Positioned ~40cm above and ~15cm behind the hand, looking forward. Simulates glasses-mounted D435.
- **Prosthesis-mounted camera** (camera_wrist): Positioned on the wrist, ~8cm from hand center, looking forward. Simulates hand-mounted D435.

To simulate **occlusion**, apply a frustum culling filter:
1. For each camera, compute the viewing frustum (D435 FOV: ~58°H × 45°V, range 0.1–1.0m).
2. For each viewpoint, keep only points visible from that camera (remove points occluded by the object itself using a depth buffer).
3. For the "high-fidelity baseline," use the complete unoccluded point cloud.

### 3.3 Hand Pose and Twist

For each object, define a canonical hand approach:
- **Pose**: Hand positioned ~15cm away from the object along the approach axis, oriented toward the object.
- **Twist**: Small forward linear velocity (0.05–0.15 m/s toward the object), near-zero angular velocity.

This gives the SMC sampler a realistic starting configuration.

---

## 4. Test Procedure

### 4.1 Latency Test (Requirement 2.4 & 1.6)

**Goal**: Measure end-to-end pipeline computation time and verify it stays under 400 ms.

**Procedure**:
- [ ] For each of the 10 objects, run `grasp_preshaping_compute()` with the multi-view point cloud.
- [ ] Record `pipeline_time_ms` from the FFI response for each invocation.
- [ ] Run each object 10 times to account for stochastic SMC initialization (different random seeds).
- [ ] Report: mean, std, min, max, P95, P99 across all 100 runs.
- [ ] Verify: P99 ≤ 400 ms (MAR), ideally P95 ≤ 100 ms (IDE).

**Additional timing breakdown** (optional but highly valuable):
- [ ] Time the ROI prediction step separately.
- [ ] Time the TSDF construction separately.
- [ ] Time each SMC iteration separately.
- This requires instrumenting the Rust code with additional timing points or extracting these steps into separate FFI calls. Given the effort, this is a nice-to-have, not a must.

### 4.2 Occlusion Proxy Test (Requirement 2.7)

**Goal**: Quantify the improvement in grasp quality from multi-view sensing vs. single-view.

**Procedure**:
- [ ] **Step A — Generate High-Fidelity Baseline**: For each object, run the pipeline with the complete unoccluded point cloud, using increased SMC samples (e.g., 100,000 instead of 20,000) and more iterations (e.g., 10 instead of 5). Record the resulting grasp type, wrist pose, and target hand position. This is the "Optimal Algorithmic Grasp" (ground truth).
- [ ] **Step B — Single-View Test**: For each object, run the pipeline using only the head-mounted camera's frustum-filtered point cloud. Use production parameters (20,000 samples, 5 iterations). Record the grasp type, wrist pose, target position.
- [ ] **Step C — Multi-View Test**: For each object, run the pipeline using both cameras' frustum-filtered point clouds (merged). Use production parameters. Record the same outputs.
- [ ] **Step D — Compute Intent Precision Δ**: For each object, compare the single-view and multi-view results against the baseline:
  - **Grasp type accuracy**: Did the predicted grasp type match the baseline? (binary per object)
  - **Wrist pose error**: Angular difference between predicted and baseline wrist orientation (degrees).
  - **Target position error**: Euclidean distance between predicted and baseline target hand position (mm).
  - **Δ = MVSS_accuracy − SingleView_accuracy** (percentage points improvement).

**Intent Precision metric (as per requirement 2.7)**:
- Define "correct prediction" as: grasp type matches baseline AND wrist pose error < 15° AND position error < 10mm.
- Compute accuracy for single-view and multi-view across all 10 objects.
- Δ = multi_view_accuracy% − single_view_accuracy%.
- Verify: Δ > 0% (MAR), Δ ≥ 10% (IDE).

---

## 5. Data to Extract

### Per-Run Data (stored in a CSV/JSON results file)

| Field | Source | Description |
|-------|--------|-------------|
| `object_id` | Test script | Which of the 10 objects |
| `run_index` | Test script | Repetition number (1–10) |
| `condition` | Test script | "baseline" / "single_view" / "multi_view" |
| `pipeline_time_ms` | `GraspComputeResponseFFI.pipeline_time_ms` | Total computation time |
| `smc_iterations_used` | `GraspComputeResponseFFI.smc_iterations_used` | Iterations before convergence |
| `grasp_type` | `GraspComputeResponseFFI.grasp_type` | 1=cyl, 2=pinch, 3=lat |
| `combined_score` | `GraspComputeResponseFFI.combined_score` | Overall grasp quality |
| `alignment_score` | `GraspComputeResponseFFI.alignment_score` | Approach alignment quality |
| `force_closure_score` | `GraspComputeResponseFFI.force_closure_score` | Force closure metric |
| `contact_score` | `GraspComputeResponseFFI.contact_score` | Dense tiered contact metric |
| `closure_amount` | `GraspComputeResponseFFI.closure_amount` | Finger closure fraction |
| `wrist_rotation_deg` | `GraspComputeResponseFFI.wrist_rotation_deg` | Planned wrist angle |
| `target_position` | `GraspComputeResponseFFI.target_px/py/pz` | Planned hand position |
| `n_cloud_points` | Test script | Number of points in input cloud |
| `n_cameras` | Test script | 1 or 2 |

### Debug Dumps (optional, for deep analysis)

- [ ] Enable `debug_visualization: true` for at least one run per condition per object.
- [ ] The resulting `.npz` files contain the full TSDF volume, all scored grasp particles across all SMC iterations, camera positions, and the ROI bounding box.
- [ ] These can be loaded into the existing `visualize_grasp_debug.py` for interactive 3D inspection.

---

## 6. Figures to Produce

### Figure 1: Pipeline Latency Distribution (Requirement 2.4)

**Type**: Box plot or violin plot.
- X-axis: Object ID (1–10).
- Y-axis: Pipeline time (ms).
- Three series: baseline (high-fidelity), single-view, multi-view.
- Horizontal dashed lines at 100 ms (IDE) and 400 ms (MAR).
- This immediately shows whether any object/condition exceeds the latency budget.

### Figure 2: Latency Summary Bar Chart

**Type**: Grouped bar chart.
- Three groups: single-view, multi-view, baseline.
- Bars: mean time, with error bars for std.
- Annotated with P95 and P99 values.
- Shows at a glance whether the IDE/MAR thresholds are met.

### Figure 3: Intent Precision Comparison (Requirement 2.7)

**Type**: Per-object bar chart.
- X-axis: Object ID (1–10).
- Y-axis: "Correct prediction" metric (1.0 = matches baseline, 0.0 = does not).
- Two series: single-view (hatched/lighter), multi-view (solid/darker).
- Annotate the overall Δ value prominently.
- This directly addresses requirement 2.7.

### Figure 4: Grasp Quality Score Comparison

**Type**: Paired bar chart or radar chart.
- For each object, show combined_score, alignment_score, force_closure_score, contact_score for single-view vs. multi-view vs. baseline.
- Demonstrates that multi-view not only matches the correct grasp type but also produces higher-quality grasps.

### Figure 5: Wrist Pose and Position Error

**Type**: Scatter plot or bar chart.
- X-axis: Object ID.
- Y-axis (left): Wrist orientation error (degrees) vs. baseline.
- Y-axis (right): Target position error (mm) vs. baseline.
- Two series per axis: single-view, multi-view.
- Shows the precision improvement from multi-view.

### Figure 6 (Optional): TSDF Visualization Comparison

**Type**: 3D surface render (using PyVista or matplotlib).
- For one representative object (e.g., the tilted cylinder), show the TSDF zero-crossing surface for:
  - (a) Single-view TSDF (visible gaps on occluded side).
  - (b) Multi-view TSDF (more complete surface).
  - (c) Baseline TSDF (fully complete).
- This is the visual evidence for *why* multi-view helps — it directly shows the backside estimation improvement.

### Table 1: Summary Results Table

A LaTeX table suitable for direct inclusion in the test document:

| Metric | Single-View | Multi-View | Baseline | MAR | IDE | Pass? |
|--------|-------------|------------|----------|-----|-----|-------|
| Mean latency (ms) | — | — | — | ≤400 | ≤100 | — |
| P99 latency (ms) | — | — | — | ≤400 | ≤100 | — |
| Grasp type accuracy (%) | — | — | 100 | — | — | — |
| Δ intent precision (%) | — | — | — | >0 | ≥10 | — |

---

## 7. Implementation Plan

### Phase 1: Test Infrastructure

- [ ] **Task 1.1**: Create a Python test script `scripts/test_software_verification.py` that loads the Rust `.so` via `ctypes` and wraps `grasp_preshaping_compute()` in a Python-friendly interface. Follow the FFI struct definitions from `src/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp` and `src/grasp_preshaping/src/c_api.rs:26-109`.
- [ ] **Task 1.2**: Implement synthetic point cloud generators for the 10 object models. Use numpy to generate surface points for cylinders, spheres, boxes, and composite shapes. Each generator returns an (N, 3) numpy array.
- [ ] **Task 1.3**: Implement the frustum culling / occlusion simulation. For each camera position and orientation, render a depth buffer of the object and keep only visible points. A simple z-buffer approach is sufficient for convex objects; for the mug, a ray-triangle test may be needed.
- [ ] **Task 1.4**: Implement the ctypes bridge to convert numpy arrays into `PointCloudViewFFI` format (matching the field layout expected by `pointcloud_view_to_pointcloud` in `c_api.rs:277-330`).

### Phase 2: Latency Test Execution

- [ ] **Task 2.1**: Run the latency test: for each of the 10 objects, call `grasp_preshaping_compute()` 10 times with the multi-view cloud. Record all response fields plus wall-clock time.
- [ ] **Task 2.2**: Repeat with single-view clouds (1 camera only).
- [ ] **Task 2.3**: Compute summary statistics (mean, std, P95, P99) and verify against 400 ms / 100 ms thresholds.
- [ ] **Task 2.4**: Generate Figure 1 (latency box plot) and Figure 2 (summary bar chart).

### Phase 3: Occlusion Proxy Test Execution

- [ ] **Task 3.1**: Generate the high-fidelity baseline: for each object, run with the complete point cloud using elevated SMC parameters (100k samples, 10 iterations). This requires creating a temporary `grasp_preshaping.yaml` override or passing config via environment variable.
- [ ] **Task 3.2**: Run single-view and multi-view tests with production parameters.
- [ ] **Task 3.3**: Compute the Intent Precision Δ metric for each object and overall.
- [ ] **Task 3.4**: Generate Figure 3 (intent precision), Figure 4 (grasp quality), Figure 5 (pose errors), and Table 1.

### Phase 4: Analysis and Reporting

- [ ] **Task 4.1**: Optionally enable debug dumps for representative objects and generate Figure 6 (TSDF comparison) using the existing `visualize_grasp_debug.py`.
- [ ] **Task 4.2**: Compile all results into the LaTeX test document format matching the template provided.
- [ ] **Task 4.3**: Write a brief analysis section interpreting the results, noting any objects where the pipeline struggles and why.

---

## 8. Verification Criteria

- [ ] **VC-2.4**: P99 pipeline latency across all objects and conditions is ≤ 400 ms (MAR pass) or ≤ 100 ms (IDE pass).
- [ ] **VC-1.6**: The pipeline autonomously produces a valid grasp command (success=1, non-zero contact_score) for all 10 objects in all conditions.
- [ ] **VC-2.7**: Multi-view intent precision Δ > 0% (MAR pass) or ≥ 10% (IDE pass) compared to single-view.
- [ ] **Reproducibility**: Running the test script twice produces identical results (set random seeds in the SMC sampler — this may require adding a seed parameter to the FFI, or accepting that the stochastic nature requires statistical comparison).

---

## 9. Potential Risks and Mitigations

1. **Risk: SMC stochasticity makes results non-reproducible.**
   - Mitigation: The Rust code uses `rand::rng()` which is thread-local and non-seedable by default. For reproducibility, either (a) run enough repetitions (10 per object) and report statistics, or (b) add an optional `seed` field to the FFI request struct. Option (a) is simpler and sufficient for a verification test.

2. **Risk: Latency measurements on development machine differ from target hardware.**
   - Mitigation: Run the test inside the Docker container (same environment as deployment). Report the hardware specs alongside the results. The 400 ms MAR has significant headroom for typical modern hardware.

3. **Risk: Frustum culling simulation is too simplistic.**
   - Mitigation: The key insight is that the test does not need to perfectly simulate real occlusion — it needs to create a *controlled* difference between single-view and multi-view inputs. Even a simple depth-buffer approach will demonstrate the principle. If the synthetic occlusion is too aggressive or too mild, adjust the camera positions to create a realistic occlusion scenario.

4. **Risk: The high-fidelity baseline may not converge to a clearly "optimal" grasp.**
   - Mitigation: Use a very high sample count (100k) and many iterations (10+). For simple geometric objects, the SMC should converge reliably. If needed, manually verify the baseline grasp by visualizing it with `visualize_grasp_debug.py`.

5. **Risk: Config override for baseline (higher SMC samples) requires rebuilding the `.so`.**
   - Mitigation: The config is loaded from `grasp_preshaping.yaml` at runtime. Simply edit the YAML file (or set `GRASP_CONFIG_PATH` to a test-specific YAML) before running the baseline. No rebuild needed.

---

## 10. Alternative Approaches

1. **ROS-level testing (launch mock.launch.py, publish to topics, measure service response time)**:
   - More realistic (tests the full ROS integration path including serialization, TF lookups, etc.).
   - Much harder to control precisely. Latency measurements include ROS overhead.
   - Better suited for a later integration test (Test 2 or 3), not this software verification test.

2. **Rust-level unit/integration tests (within Cargo test framework)**:
   - Most precise latency measurements (no Python ctypes overhead).
   - Harder to generate the synthetic dataset and produce the figures.
   - Could be used to supplement the Python test for the latency-critical path.

3. **Using real D435 recordings (bag files) instead of synthetic clouds**:
   - Most realistic point clouds (real noise, real occlusion patterns).
   - Requires recorded bag files from the actual camera setup.
   - Harder to establish ground truth.
   - Better suited for Test 2 (Hardware Verification) where real sensor data is available.

**Recommendation**: Proceed with the ctypes approach (primary plan). It offers the best balance of realism (tests the actual compiled Rust planner), ease (Python script, no ROS needed), and controllability (synthetic data with known ground truth).

---

## 11. Balancing Realism and Ease — Reasoning

The test document specifies "synthetic dataset" and "simulated viewpoints," which already signals that perfect physical realism is not the goal. The purpose is to isolate the *computational advantage* of multi-view sensing. Here is the reasoning for each design choice:

| Decision | Realism Cost | Ease Benefit | Justification |
|----------|-------------|-------------|---------------|
| Call `.so` directly (no ROS) | Misses middleware overhead (~5-20 ms) | Eliminates setup complexity, removes jitter | Middleware overhead is constant and small; the algorithmic latency dominates |
| Synthetic point clouds (not real scans) | No sensor noise, no quantization artifacts | Perfect ground truth, reproducible | Sensor noise affects all conditions equally; the Δ metric cancels it out |
| Simple z-buffer occlusion | Not perfectly realistic occlusion | Trivial to implement | The test needs *some* occlusion difference between 1-view and 2-view, not perfect simulation |
| 10 canonical objects | Limited diversity | Easy to analyze, clear per-object narrative | 10 objects is specified in the test document; the selection covers the 3 grasp types well |
| 10 repetitions per object | Does not capture long-tail behavior | Sufficient for P95/P99 estimates | 100 total runs per condition gives reasonable statistics |
| ctypes bridge | ~1-2 ms Python overhead per call | No compilation needed | The overhead is negligible relative to the 100-400 ms budget |

The overall philosophy: **measure the algorithm under controlled conditions, not the system under realistic conditions**. System-level testing belongs in later test phases.
