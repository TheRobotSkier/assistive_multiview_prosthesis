# Per-Stage Latency Benchmark for Test 1 Software Verification (v3)

## Objective

Create a **non-invasive, toggleable** per-stage latency benchmark that records wall-clock arrival times of messages on intermediate pipeline topics. This decomposes the EMG-to-motor-command path into measurable segments without modifying production pipeline code (except one optional 1-line addition).

Satisfies **Requirement 2.4**: "Maximum computation time from EMG signal to autonomous motor command output" — MAR ≤ 400 ms, IDE ≤ 100 ms.

## Approach: Arrival-Time Tapping

The test script subscribes to intermediate topics and records `time.perf_counter()` when each message arrives. Deltas between consecutive arrivals give per-stage latency.

**Why this works:**
- **Zero production code changes** (except optional 1-line for Rust timing)
- **Fully toggleable** — only runs when this test is invoked
- **No clock-domain issues** — all timestamps from the same `perf_counter` in the test process
- **Measures real wall-clock latency** including ROS middleware overhead

### Pipeline Critical Path and Observable Topics

```
Test publishes approach trajectory to /hand_pose ──────────── (preparation)
Test publishes cloud + pose + twist ───────────────────────── (preparation)
Test publishes EMG POWER gesture ──────────────────────────── t₀
  │
  ▼
/pipeline/state → SEGMENTING (1) ─────────────────────────── t₁
  │   Stage A: Pipeline Manager overhead = t₁ - t₀
  │
  ▼
/segmentation/click_positive ──────────────────────────────── t₂
  │   Stage B: Twist propagation collision + click = t₂ - t₁
  │
  ▼
/segmentation/object_cloud ────────────────────────────────── t₃
  │   Stage C: Segmentation inference = t₃ - t₂
  │
  ▼
/pipeline/state → PLANNING (2) ───────────────────────────── t₄
  │   Stage D: Pipeline manager cloud handling = t₄ - t₃
  │
  ▼
/grasp_preshaping/grasp_type ─────────────────────────────── t₅
  │   Stage E: Grasp preshaping (Rust + ROS) = t₅ - t₄
  │
  ──────────────────────────────────────────────────────────
  TOTAL = t₅ - t₀
```

## Implementation Plan

### Part A: Create the Per-Stage Latency Test Script

- [ ] **A1. Create `tests/test1_software_verification/run_latency_stages.py`**
  - New standalone test script, same style as `run_tier_b.py`
  - Requires the mock launch running: `ros2 launch prosthesis_launch mock.launch.py`
  - Usage: `python run_latency_stages.py [--objects ...] [--repetitions 30]`
  - Rationale: Self-contained benchmark that produces per-stage latency data without affecting any other test

- [ ] **A2. Implement multi-topic arrival-time subscriber**
  - Subscribe to these 5 topics with callbacks recording `time.perf_counter()`:
    | Topic | Type | Event Detected |
    |-------|------|---------------|
    | `/pipeline/state` | `Int32` | State transitions (0→1 = SEGMENTING, 1→2 = PLANNING) |
    | `/segmentation/click_positive` | `PointStamped` | Twist propagation fired |
    | `/segmentation/object_cloud` | `PointCloud2` | Segmentation completed |
    | `/grasp_preshaping/grasp_type` | `Int32` | Grasp planner output |
    | `/grasp_preshaping/target_finger_closures` | `Float64MultiArray` | Motor command |
  - Each callback stores `(topic, perf_counter_time, message_data)` into a per-trial event list protected by `threading.Lock`
  - Rationale: These 5 topics cover every stage boundary. Arrival-time deltas give per-stage latency.

- [ ] **A3. Implement approach trajectory publisher for twist propagation**
  - The mock launch does NOT include a hand pose publisher. Twist propagation requires ≥2 poses on `/hand_pose` with linear velocity > 0.02 m/s to detect collisions.
  - The test must publish an approach trajectory to `/hand_pose` before the EMG trigger:
    1. Load the object's approach pose from `hand_approaches.py`
    2. Start from a position ~30 cm behind the approach point (along the approach direction)
    3. Publish a series of `PoseStamped` messages moving toward the object at ~0.15 m/s
    4. The trajectory must pass through (or near) the object's point cloud so twist propagation detects a collision
  - Implementation: publish poses at 10 Hz for ~2 seconds before the EMG trigger, building up a pose buffer with consistent velocity
  - Rationale: Without this, twist propagation cannot fire in the mock setup, and the test would only measure the alternative pipeline manager direct path

- [ ] **A4. Implement the trial execution loop**
  - For each object × repetition:
    1. **Clear** previous trial events
    2. **Publish** synthetic cloud to `/segmentation/object_cloud` (or `/fused_pointcloud` depending on mock wiring), pose to `/hand_pose`, twist to `/hand_twist`
    3. **Publish approach trajectory** to `/hand_pose` for ~2 seconds to build pose buffer
    4. **Wait** for twist propagation to have ≥2 poses and a fresh cloud
    5. **Record** `t₀ = time.perf_counter()`
    6. **Publish** EMG POWER gesture (`Int32(data=1)`) on `/emg/gesture_label`
    7. **Wait** for `/grasp_preshaping/grasp_type` to arrive (15s timeout)
    8. **Extract** all event timestamps from the trial
    9. **Publish** EMG OPEN gesture (`Int32(data=3)`) to reset pipeline to IDLE
    10. **Wait** 500ms for pipeline reset
  - Rationale: Mirrors the existing Tier B pattern but adds intermediate topic tapping and approach trajectory

- [ ] **A5. Compute per-stage latencies from arrival times**
  - For each trial, extract timestamps and compute:
    | Stage | Computation | Description |
    |-------|------------|-------------|
    | `pipeline_manager` | `t₁ - t₀` | EMG receipt → state transition to SEGMENTING |
    | `twist_propagation` | `t₂ - t₁` | State transition → click published |
    | `segmentation` | `t₃ - t₂` | Click received → segmented cloud published |
    | `pm_cloud_handling` | `t₄ - t₃` | Segmented cloud → PLANNING state |
    | `preshaping` | `t₅ - t₄` | PLANNING → grasp_type published |
    | `total` | `t₅ - t₀` | EMG trigger → grasp command |
  - Handle edge cases:
    - If `/segmentation/click_positive` never arrives (twist propagation didn't fire), record `twist_propagation_ms = NaN` and note `path = "direct"` (pipeline manager bypassed twist propagation)
    - If `/pipeline/state` skips PLANNING, record the applicable stages as `NaN`
  - Rationale: This decomposition identifies which stage dominates the latency budget

- [ ] **A6. Output results to CSV**
  - Write `results/latency_per_stage_results.csv` with per-trial data:
    ```
    object, repetition, path, pipeline_manager_ms, twist_propagation_ms,
    segmentation_ms, pm_cloud_handling_ms, preshaping_ms, total_ms, status
    ```
  - Write `results/latency_per_stage_summary.csv` with mean/P95/P99 per object per stage
  - Rationale: Long-format CSV for analysis, summary CSV for quick comparison

### Part B: Add `pipeline_time_ms` to Service Response (1-Line Change)

- [ ] **B1. Append `pipeline_time_ms` to the Trigger response message**
  - In `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:503-512`, add one line:
    - After the existing `response->message +=` block, append:
      `", pipeline_time_ms=" + std::to_string(ffi_response.pipeline_time_ms)`
  - This makes the Rust algorithmic time available to any service caller
  - The test script parses `pipeline_time_ms=XX` from the response to decompose Stage E into:
    - `rust_compute_ms` = the parsed `pipeline_time_ms` value
    - `ros_service_overhead_ms` = `preshaping_ms - rust_compute_ms`
  - Rationale: Without this, we cannot separate Rust compute time from ROS service overhead in the preshaping stage. This is the **only** production code change, and it is purely additive — it does not change the response format for existing consumers that don't parse the message string.

### Part C: Integrate with Existing Test Infrastructure

- [ ] **C1. Update `plot_results.py` to visualize per-stage latency**
  - Add a stacked bar chart: each bar = one object, stacked segments = per-stage latency
  - Add a horizontal line at 400 ms (MAR threshold)
  - Add a waterfall/cascade chart showing cumulative latency from EMG to command
  - Rationale: Visual breakdown makes it immediately clear which stage dominates

- [ ] **C2. Update `run_tier_b.py` to capture `pipeline_time_ms` from service response**
  - In Method B, parse `pipeline_time_ms=XX` from the response message (once B1 is done)
  - Add `pipeline_time_ms` and `ros_overhead_ms` columns to the Tier B output CSV
  - Rationale: Enriches existing Tier B data with zero additional test runs

- [ ] **C3. Generate LaTeX table for the report**
  - Produce a table matching the test specification format:
    ```
    | Stage | Mean (ms) | P95 (ms) | P99 (ms) | % of Total |
    ```
  - Include MAR/IDE pass/fail verdicts

### Part D: Validation

- [ ] **D1. Verify the mock launch produces the expected topic sequence**
  - Run the mock launch, inject approach trajectory + EMG trigger, confirm all 5 intermediate topics fire in the expected order
  - Specifically verify that twist propagation detects a collision when the approach trajectory intersects the object cloud
  - If the mock cloud publisher publishes to `/camera/depth/color/points` but twist propagation subscribes to `/fused_pointcloud`, verify there is a topic bridge or that the test must publish to `/fused_pointcloud` directly

- [ ] **D2. Run the full benchmark on the 10-object synthetic dataset**
  - 30 repetitions per object
  - Verify total latency is consistent with existing Tier B measurements
  - Verify the sum of per-stage times approximately equals the total (within 15%)

- [ ] **D3. Cross-validate with existing data**
  - Compare `preshaping_ms` against the existing Tier A `pipeline_time_ms` (37–73 ms)
  - The difference should approximate ROS service overhead
  - Compare total latency against the existing Tier B service-method measurements (118–288 ms)

## Verification Criteria

- [ ] Per-stage latency for all pipeline stages from EMG trigger to grasp command
- [ ] Only 1-line production code change (optional `pipeline_time_ms` in service response)
- [ ] Fully toggleable — does not affect other tests
- [ ] Results for all 10 synthetic objects with ≥ 30 repetitions each
- [ ] Per-stage P95 values reported against 400 ms MAR budget
- [ ] Total P95 latency matches existing Tier B measurements within 15%
- [ ] CSV output compatible with existing `plot_results.py`

## Potential Risks and Mitigations

1. **Twist propagation may not fire if approach trajectory is wrong**
   Mitigation: The test script controls the approach trajectory — it can compute a straight-line path from a start position through the object's bounding box. The velocity must exceed 0.02 m/s and the time-to-hit must exceed 0.4s. The test logs whether twist propagation fired and falls back to measuring the direct path if it didn't.

2. **Mock cloud topic mismatch**
   The mock cloud publisher publishes to `/camera/depth/color/points` but twist propagation subscribes to `/fused_pointcloud`. The test must publish the synthetic cloud to the correct topic. Check the mock launch for any remapping. If no bridge exists, the test publishes directly to `/fused_pointcloud`.

3. **Segmentation inference server not running**
   Mitigation: Check for the inference server at startup (`http://127.0.0.1:5678/health`). If unavailable, the test skips the segmentation stage and measures only the stages that work, marking the rest as "requires inference server."

4. **The 150 ms preshaping call delay is a real latency component**
   This is not measurement error — it's a deliberate delay in `twist_propagation_node.py:478`. The per-stage breakdown will make this visible as part of the `preshaping` stage.

5. **Topic ordering within a single ROS spin**
   All callbacks are serialized within the test node's executor, so arrival-time ordering is deterministic from the test's perspective. No race conditions.

## Files to Create/Modify

| File | Action | Description |
|------|--------|-------------|
| `tests/test1_software_verification/run_latency_stages.py` | **Create** | Per-stage latency benchmark script |
| `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | **Modify** (1 line) | Append `pipeline_time_ms` to service response |
| `tests/test1_software_verification/plot_results.py` | **Modify** | Add per-stage visualization charts |
| `tests/test1_software_verification/run_tier_b.py` | **Modify** | Parse `pipeline_time_ms` from service response |
