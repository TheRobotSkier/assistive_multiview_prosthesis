# Test 1: Software Verification — Revised Plan (v2)

## Objective

Design and execute an offline software verification test that validates the core grasp planning pipeline using synthetic point cloud data. The test must produce quantitative evidence for requirements 1.6, 2.4, and 2.7.

---

## 1. Revised Latency Strategy: Two-Tier Measurement

### The Problem with Pure `.so` Timing

Your instinct is correct. The `.so` only measures the grasp planner (ROI prediction → TSDF → SMC → scoring). But requirement 2.4 says "Maximum computation time from EMG signal to autonomous motor command output." The full path includes:

| Stage | Component | Where Time Goes |
|-------|-----------|----------------|
| 1. EMG trigger | `emg_bridge` | Gesture classification (fast, ~5-10ms) |
| 2. Twist estimation | `twist_propagation` | Accumulates pose history, estimates velocity (~1ms per cycle) |
| 3. Click generation | `twist_propagation` | Propagation + KDTree intersection (~5-15ms) |
| 4. **Segmentation** | `inference_server` + `segmentation_bridge` | **MinkowskiEngine forward pass over HTTP (~100-500ms)** |
| 5. **Grasp planning** | `grasp_preshaping` `.so` | **ROI + TSDF + SMC (~50-200ms)** |
| 6. Motor command output | `preshaping_service_bridge_node` | Publish joint commands (~1ms) |

The segmentation is likely the **dominant cost** — it's a MinkowskiEngine sparse 3D CNN running on CPU (`device = torch.device("cpu")` per `src/segmentation/nodes/inference_server.py:36`), communicated over HTTP with base64-encoded payloads. This could easily be 200-500ms.

### Recommended Two-Tier Approach

**Tier A — Isolated Component Timing (ctypes, no ROS)**

This measures the grasp planner in isolation. It answers: "Is the algorithm fast enough on its own?" This is what v1 of the plan proposed.

- Call `grasp_preshaping_compute()` directly via ctypes
- Record `pipeline_time_ms` from the FFI response
- This gives a clean, reproducible measurement of the planner alone

**Tier B — Full Pipeline Timing (ROS integration test)**

This measures the complete EMG-to-command path. It answers: "Does the full system meet the 400ms budget?"

- Launch the mock pipeline (`mock.launch.py`)
- Publish synthetic EMG gestures + point clouds
- Timestamp the EMG trigger and the motor command output
- Measure wall-clock end-to-end latency

For the **latency test (2.4)**, you should do both tiers. Tier A gives you the algorithmic budget breakdown. Tier B gives you the real number. If Tier B fails, Tier A tells you which component to optimize.

**However**, for the **occlusion proxy test (2.7)**, the segmentation is not the bottleneck — it is the same in both single-view and multi-view conditions (the segmentation operates on the same scene cloud regardless). The difference between single-view and multi-view only manifests in the grasp planner, where the TSDF quality changes. So for the occlusion test, calling the `.so` directly is perfectly appropriate.

### Practical Implication

- **Latency test**: Run both Tier A (ctypes, fast, many repetitions) and Tier B (ROS, fewer repetitions, but captures the full budget).
- **Occlusion test**: Run Tier A only (ctypes). Segmentation is not a variable here — the test compares grasp quality given different point cloud inputs, not segmentation performance.

---

## 2. Revised Object Selection: More Complex Shapes

### Why Simple Objects Are Insufficient

Simple primitives (perfect cylinders, spheres, boxes) have two problems:

1. **They don't produce meaningful occlusion differences.** A sphere looks the same from every angle — single-view and multi-view will produce nearly identical TSDFs. The Δ will be ~0% for spheres and barely measurable for upright cylinders.

2. **They don't stress the superquadric backside estimation.** The superquadric fitting (3 templates: sphere, box, cylinder) will trivially match a sphere to a sphere. The real test is whether it can reconstruct the backside of an irregular object from partial views.

### Revised Object Set

Use a mix of **parametric objects** (easy to generate, known ground truth) and **mesh-based objects** (more realistic, available from public datasets).

| # | Object | Source | Rationale | Expected Grasp | Occlusion Sensitivity |
|---|--------|--------|-----------|----------------|----------------------|
| 1 | Cylinder (upright, r=3cm, h=12cm) | Parametric | Baseline, easy to analyze | Cylindrical | Low |
| 2 | Tilted cylinder (30° off vertical) | Parametric | Orientation robustness | Cylindrical | Medium |
| 3 | **Ellipsoid** (a=5, b=3, c=3 cm) | Parametric | Asymmetric, different from every angle | Cylindrical | Medium |
| 4 | **Tapered bottle** (r_bottom=3.5, r_top=2.5, h=18cm) | Parametric | Real-world shape, non-uniform cross-section | Cylindrical | High |
| 5 | **L-shaped block** (6×3×3 + 3×3×6 cm) | Parametric | Concave region, self-occluding | Cylindrical | High |
| 6 | **Mug with handle** | Mesh (YCB or custom) | Complex topology, handle occlusion | Cylindrical/Pinch | Very High |
| 7 | **Banana** (curved, elongated) | Mesh (YCB #11) | Curved shape, significant self-occlusion | Cylindrical | High |
| 8 | **Drill** (elongated + handle) | Mesh (YCB #35) | Complex industrial shape | Cylindrical | High |
| 9 | **Small cube** (2.5cm side) | Parametric | Small object, pinch target | Pinch | Low |
| 10 | **Credit card / thin book** (8.5×5.4×0.1 cm) | Parametric | Very flat, lateral target | Lateral | Medium |

### How to Get Mesh-Based Objects

- **YCB Object Set**: The Yale-Cali-Berkeley dataset provides high-quality meshes freely available at `ycb-benchmarks.s3-website-us-east-1.amazonaws.com`. Objects like the banana (#11), mug (#16), and power drill (#35) are standard benchmarks.
- **Download the `.obj` files**, sample surface points uniformly (~10,000 points per object), and use them as synthetic point clouds.
- For objects not in YCB (L-shaped block, tapered bottle), generate parametrically with numpy.

### Why This Matters for the Occlusion Test

Objects 4-8 are specifically chosen because they have **significant self-occlusion** when viewed from one side. The mug handle is invisible from the front; the drill's handle is occluded by its body; the banana curves away. These are the objects where multi-view will show the largest Δ, making the test results meaningful rather than trivially "everything is the same."

---

## 3. Segmentation in the Occlusion Test: Not Needed

### Why

The occlusion proxy test (requirement 2.7) compares **grasp planning quality** between single-view and multi-view point clouds. The segmentation step is not part of this comparison because:

1. **Segmentation operates on the full scene cloud**, not on individual views. The segmentation node receives `/segmentation/input_cloud` (a single merged cloud from one or both cameras) and produces a foreground mask. The quality of segmentation depends on the click placement and the model, not on the number of views.

2. **The test uses pre-segmented synthetic point clouds.** Since we generate the object point clouds directly (no background, no need for segmentation), the segmentation step is bypassed entirely. We feed the object point cloud straight to the grasp planner.

3. **Other tests cover segmentation more directly.** As you noted, there are (or will be) other tests that specifically evaluate segmentation accuracy, latency, and robustness. Adding segmentation to this test would conflate two independent variables.

### What the Occlusion Test Actually Measures

```
Input:  Pre-segmented object point cloud (synthetic, with controlled occlusion)
  ↓
Process: TSDF construction + superquadric backside estimation + SMC grasp planning
  ↓
Output: Grasp type, wrist pose, target position, quality scores
```

The independent variable is the **number of camera viewpoints** used to generate the point cloud (1 vs. 2). The dependent variable is **how close the resulting grasp is to the high-fidelity baseline**.

---

## 4. Revised Implementation Plan

### Phase 1: Test Infrastructure

- [ ] **Task 1.1**: Create `scripts/test_software_verification.py` — main test orchestrator. Parses command-line args for which sub-test to run, loads the `.so`, manages results.
- [ ] **Task 1.2**: Implement ctypes bridge to `grasp_preshaping_compute()`. Define Python `ctypes.Structure` classes matching `GraspComputeRequestFFI` and `GraspComputeResponseFFI` from `src/grasp_preshaping/include/grasp_preshaping/ffi_types.hpp:20-121` and `src/grasp_preshaping/src/c_api.rs:26-109`.
- [ ] **Task 1.3**: Implement parametric point cloud generators (cylinder, ellipsoid, tapered bottle, L-block, cube, thin plate). Each returns an (N, 3) float32 numpy array with ~5,000-10,000 surface points.
- [ ] **Task 1.4**: Download and process YCB mesh objects (banana, mug, drill). Sample surface points from `.obj` files using `trimesh` or `open3d`.
- [ ] **Task 1.5**: Implement the frustum culling / depth-buffer occlusion simulator. For each camera position, render a depth image of the object and keep only visible points. Use a simple z-buffer approach (rasterize object points into a grid, keep only the nearest per pixel).

### Phase 2: Latency Test (Tier A — ctypes, planner only)

- [ ] **Task 2.1**: For each of the 10 objects, call `grasp_preshaping_compute()` 10 times with multi-view clouds. Record `pipeline_time_ms` and all response fields.
- [ ] **Task 2.2**: Repeat with single-view clouds.
- [ ] **Task 2.3**: Compute statistics (mean, std, P95, P99) and verify against thresholds.
- [ ] **Task 2.4**: Generate Figure 1 (latency box plot) and Figure 2 (summary bar chart).

### Phase 3: Latency Test (Tier B — ROS, full pipeline)

- [ ] **Task 3.1**: Create a ROS integration test script `scripts/test_pipeline_latency.py` that:
  - Launches `mock.launch.py` with the segmentation inference server running
  - Publishes synthetic EMG gesture triggers to `/emg/gesture_label`
  - Publishes synthetic point clouds to `/segmentation/input_cloud` and click points to `/segmentation/click_positive`
  - Subscribes to `/grasp_preshaping/target_finger_closures` and `/grasp_preshaping/wrist_pose`
  - Measures wall-clock time from EMG publish to motor command receipt
- [ ] **Task 3.2**: Run for all 10 objects, 5 repetitions each (fewer due to overhead).
- [ ] **Task 3.3**: Report the full-pipeline latency alongside the planner-only latency. If the total exceeds 400ms, break down which stages consume the budget.

### Phase 4: Occlusion Proxy Test (Tier A — ctypes)

- [ ] **Task 4.1**: Generate high-fidelity baselines: for each object, run with complete unoccluded point cloud, elevated SMC parameters (100k samples, 10 iterations). Use a separate `grasp_preshaping_baseline.yaml` config file.
- [ ] **Task 4.2**: Run single-view tests: for each object, use head-mounted camera frustum only, production parameters.
- [ ] **Task 4.3**: Run multi-view tests: for each object, use both camera frustums, production parameters.
- [ ] **Task 4.4**: Compute Intent Precision Δ: compare single-view and multi-view results against the baseline for grasp type, wrist pose, and target position.
- [ ] **Task 4.5**: Generate Figure 3 (intent precision), Figure 4 (grasp quality), Figure 5 (pose errors), and Table 1.

### Phase 5: Analysis and Reporting

- [ ] **Task 5.1**: Optionally enable debug dumps for representative objects and generate Figure 6 (TSDF comparison).
- [ ] **Task 5.2**: Compile results into LaTeX test document.
- [ ] **Task 5.3**: Write analysis, noting which objects benefit most from multi-view and why.

---

## 5. Data to Extract

### Per-Run Data

Same as v1, with the addition of:

| Field | Source | Description |
|-------|--------|-------------|
| `test_tier` | Test script | "A" (planner only) or "B" (full pipeline) |
| `wall_clock_ms` | Python `time.perf_counter()` | Wall-clock time (Tier B only) |
| `segmentation_time_ms` | Instrumented segmentation node | Segmentation inference time (Tier B only) |

### Summary Data (for the results table)

| Metric | Tier A (Planner) | Tier B (Full Pipeline) | MAR | IDE |
|--------|-------------------|------------------------|-----|-----|
| Mean latency (ms) | — | — | ≤400 | ≤100 |
| P99 latency (ms) | — | — | ≤400 | ≤100 |
| Segmentation contribution (ms) | N/A | — | — | — |
| Planner contribution (ms) | — | — | — | — |

---

## 6. Figures (Revised)

### Figure 1: Latency Budget Breakdown (Tier A + Tier B combined)

**Type**: Stacked bar chart or waterfall chart.
- For each object, show the latency broken into components:
  - Segmentation (from Tier B)
  - Grasp planner (from Tier A)
  - ROS overhead (Tier B minus Tier A minus segmentation)
- Horizontal dashed lines at 100ms and 400ms.
- This is the single most informative figure for requirement 2.4.

### Figure 2: Planner Latency Distribution (Tier A detail)

**Type**: Box plot.
- X-axis: Object ID (1-10).
- Y-axis: Planner time (ms).
- Two series: single-view, multi-view.
- Shows whether point cloud density (multi-view has more points) affects planner latency.

### Figure 3: Intent Precision (Requirement 2.7)

**Type**: Per-object grouped bar chart.
- X-axis: Object ID.
- Y-axis: "Correct prediction" metric vs. baseline.
- Two series: single-view, multi-view.
- Annotate Δ prominently.
- **Expectation**: Objects 4-8 (bottle, L-block, mug, banana, drill) should show the largest Δ. Objects 1, 3, 9 (simple shapes) should show small or zero Δ.

### Figure 4: Grasp Quality Score Comparison

**Type**: Radar chart or grouped bar chart.
- Score components for each object: alignment, force closure, contact score.
- Three conditions: baseline, single-view, multi-view.

### Figure 5: Wrist Pose and Position Error vs. Baseline

**Type**: Bar chart with error bars.
- Per-object, show angular error (deg) and position error (mm).
- Two series: single-view, multi-view.

### Figure 6: TSDF Surface Comparison (Optional, for one object)

**Type**: 3D render.
- Pick an object with high occlusion sensitivity (e.g., the mug or L-block).
- Show TSDF zero-crossing surface for single-view, multi-view, and baseline side by side.
- Visual proof of *why* multi-view helps.

### Table 1: Summary Results

Same as v1, but with the addition of the full-pipeline latency column.

---

## 7. Potential Risks and Mitigations

1. **Risk: Segmentation server not available for Tier B testing.**
   - Mitigation: The segmentation runs in a separate Docker container (`docker-compose.yml` service `segmentation`). Build and start it separately. If GPU is not available, it runs on CPU (as it currently does per `inference_server.py:36`). Expect 200-500ms on CPU.

2. **Risk: YCB meshes require additional download/processing steps.**
   - Mitigation: Cache the downloaded meshes in the repo under `data/test_objects/`. Provide a download script. Alternatively, fall back to purely parametric objects if mesh processing is too cumbersome — the tapered bottle and L-block can substitute for the mug and drill in terms of occlusion sensitivity.

3. **Risk: Tier B latency measurements are noisy due to ROS scheduling.**
   - Mitigation: Run inside Docker with `network_mode: host` (already configured). Use `time.perf_counter()` for sub-millisecond resolution. Run 5-10 repetitions and report P95/P99. Accept that ROS overhead adds ~10-30ms of variance.

4. **Risk: The high-fidelity baseline doesn't converge for complex objects.**
   - Mitigation: Increase baseline samples to 200k and iterations to 15. For objects where it still doesn't converge, manually inspect the debug dump with `visualize_grasp_debug.py` and adjust the hand approach pose.

5. **Risk: Frustum culling simulation doesn't produce realistic enough occlusion.**
   - Mitigation: For mesh-based objects, use proper ray-triangle intersection (via `trimesh` or `pyembree`). For parametric objects, the z-buffer approach is sufficient since the geometry is simple.

---

## 8. Segmentation Decision Summary

| Question | Answer | Reasoning |
|----------|--------|-----------|
| Include segmentation in latency test? | **Yes (Tier B)** | Segmentation is likely the largest latency contributor. Must measure it to validate the 400ms budget. |
| Include segmentation in occlusion test? | **No** | The test compares grasp quality given different point cloud inputs. Segmentation is not a variable. Other tests cover it. |
| Need the inference server running? | **Only for Tier B** | Tier A (ctypes) bypasses segmentation entirely. |

---

## 9. Object Complexity Decision Summary

| Approach | Pros | Cons | Verdict |
|----------|------|------|---------|
| All parametric (v1) | Easy to generate, perfect ground truth | Spheres/cylinders show zero occlusion difference | Insufficient for 2.7 |
| All YCB meshes | Most realistic | Download dependency, harder to establish ground truth | Good but heavy |
| **Mixed (v2)** | **Parametric for known shapes + meshes for complex occlusion** | **Two code paths for generation** | **Best balance** |

The mixed approach ensures at least 5 objects (4-8) will demonstrate meaningful multi-view improvement, while the parametric objects (1-3, 9-10) provide clean baselines and cover all three grasp types.
