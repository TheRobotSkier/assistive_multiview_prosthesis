# Per-Stage Latency Benchmark for Test 1 Software Verification

## Objective

Design and implement a per-stage latency benchmark that decomposes the full EMG-to-motor-command pipeline into individually measured stages. This will:

1. Satisfy **Requirement 2.4**: "Maximum computation time from EMG signal to autonomous motor command output" — measured against MAR ≤ 400 ms and IDE ≤ 100 ms
2. Replace the current single-number `pipeline_time_ms` (which only covers the Rust grasp planner) with a complete breakdown of every pipeline stage
3. Enable identification of latency bottlenecks for optimization
4. Provide statistically rigorous per-stage measurements (mean, P95, P99) across the 10-object synthetic dataset

## Current State of Latency Measurement

| Stage | Currently Measured? | How |
|-------|-------------------|-----|
| Rust grasp planner (ROI + TSDF + SMC + scoring) | Yes | `pipeline_time_ms` in FFI response (`src/grasp_preshaping/src/c_api.rs:362,599`) |
| End-to-end ROS service call | Partially | Tier B test (`tests/test1_software_verification/run_tier_b.py:273-286`) — 25 measurements, service method only |
| End-to-end EMG path | Partially | Tier B Method A — measures total wall-clock but has no per-stage breakdown |
| Twist propagation (collision detection cycle) | No | — |
| Segmentation (HTTP inference round-trip) | No | — |
| Point cloud fusion (per-cycle) | No | — |
| ROS middleware overhead per hop | No | — |
| Pipeline manager state machine overhead | No | — |

## Pipeline Stages Requiring Measurement

Based on the test specification, the latency test measures "from EMG signal to autonomous motor command output." The critical path is:

```
EMG trigger → Pipeline Manager (IDLE→SEGMENTING) → Twist Propagation activation →
Collision detection cycle → Click publishing → Segmentation inference →
Segmented cloud receipt → Preshaping service call → Rust compute → 
Motor command publication
```

The stages to measure individually are:

### Stage 1: Pipeline Manager Overhead
- EMG gesture receipt → state transition to SEGMENTING → twist propagation activation service call
- Source: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:238-245`

### Stage 2: Twist Propagation Collision Detection
- One cycle: twist estimation → pose propagation → KDTree collision check → click cluster publishing
- Source: `src/twist_propagation/twist_propagation/twist_propagation_node.py:1179-1380`
- Config: cycle_delay_s = 0.1s, but the actual computation time per cycle is unknown

### Stage 3: Segmentation Inference
- Click accumulation debounce → base64 encode → HTTP POST to MinkowskiEngine server → mask decode → cloud extraction
- Source: `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:234-281`
- This is likely the **largest unknown latency contributor** (network + GPU inference)

### Stage 4: Grasp Preshaping (Rust)
- Already measured: `pipeline_time_ms` covers ROI prediction → cloud pruning → TSDF construction → SMC optimization → grasp scoring
- Source: `src/grasp_preshaping/src/c_api.rs:361-624`
- Current data: 37–73 ms (median ~50 ms) from `latency_results.csv`

### Stage 5: ROS Middleware Overhead
- Serialization + DDS transport per topic hop
- Measured as the delta between Tier B service method (includes ROS) and Tier A FFI (excludes ROS)

### Stage 6: End-to-End Total
- Sum of all stages (or direct wall-clock measurement from EMG injection to grasp_type output)

## Implementation Plan

### Part A: Add Per-Stage Timing Instrumentation to Pipeline Nodes

- [ ] **A1. Add timing markers to twist propagation node** (`src/twist_propagation/twist_propagation/twist_propagation_node.py`)
  - Add `time.perf_counter()` markers around the key computations in `_run_idle_cycle()`:
    - Twist estimation (`_estimate_twist()`) — line ~1210
    - Pose propagation + collision detection (`_propagate_and_find_hit()`) — line ~1261
    - Click cluster generation + publishing — lines ~1332-1356
  - Publish timing data on a new topic `/twist_propagation/timing` (or include in the existing `/twist_propagation/status` JSON)
  - Rationale: Twist propagation runs at 10 Hz cycles; knowing how much of that 100 ms budget is consumed by computation vs. idle waiting is critical

- [ ] **A2. Add timing markers to segmentation bridge node** (`src/segmentation/segmentation_bridge/segmentation_ros2_node.py`)
  - Add `time.perf_counter()` markers around the key steps in `_run_inference()`:
    - Base64 encoding of xyz + rgb — lines ~253-258
    - HTTP POST round-trip (`requests.post()`) — line ~262
    - Mask decode + foreground extraction — lines ~264-279
  - Publish timing data on a new topic `/segmentation/timing` (Float64MultiArray or JSON on existing diagnostics topic)
  - Rationale: The HTTP inference round-trip is the **single largest unknown** in the latency budget. Without measurement, we cannot determine if segmentation or the Rust planner is the bottleneck.

- [ ] **A3. Add timing markers to point cloud fusion node** (`src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`)
  - Add `time.perf_counter()` markers around each step in `_process_clouds()`:
    - TF2 transform — lines ~416-438
    - Concatenation — lines ~440-474
    - Distance filter — lines ~484-499
    - Bbox removal — lines ~504-541
    - Voxel downsampling — lines ~546-548
    - Build + publish — lines ~553-568
  - Include timing in the existing periodic stats log (every 10s)
  - Rationale: Fusion runs continuously at 15 Hz; if a single cycle takes >66 ms it would miss frames. Knowing per-step cost enables optimization.

- [ ] **A4. Add timing markers to pipeline manager node** (`src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`)
  - Measure wall-clock time from `_on_emg_gesture()` entry to `_activate_twist_propagation()` service call — lines ~220-245
  - Measure time from `_on_object_cloud()` entry to `_request_preshaping()` service call — lines ~264-273
  - Include in the existing state transition log
  - Rationale: The state machine overhead should be negligible (<1 ms) but needs to be confirmed.

- [ ] **A5. Add sub-step timing to Rust grasp planner** (`src/grasp_preshaping/src/c_api.rs`)
  - Add `std::time::Instant::now()` markers around the major sub-steps in `compute_from_request()`:
    - ROI prediction — lines ~374-380
    - Cloud pruning — line ~382
    - TSDF construction — lines ~396-414
    - SMC optimization loop — lines ~418+
    - Grasp scoring — parallel scoring across 3 types
  - Add these sub-timings as new fields in `GraspComputeResponseFFI` (or a separate timing struct)
  - Rationale: The Rust planner is already the best-instrumented stage, but knowing the breakdown (TSDF vs. SMC vs. scoring) would guide optimization efforts.

### Part B: Create Per-Stage Latency Benchmark Test Script

- [ ] **B1. Create `tests/test1_software_verification/run_latency_stages.py`**
  - This is the main benchmark script that runs each pipeline stage in isolation using synthetic data, similar to how `run.py` calls the FFI bridge directly
  - Structure:
    ```
    Stage 1: Twist propagation collision detection (offline)
      - Load synthetic point cloud + pose trajectory
      - Run _estimate_twist() + _propagate_and_find_hit() in a loop
      - Measure per-cycle computation time (excluding idle waits)
    
    Stage 2: Segmentation inference (requires inference server)
      - Load synthetic cloud + clicks
      - Call inference server HTTP endpoint directly
      - Measure encode + HTTP + decode time
    
    Stage 3: Grasp preshaping (already exists via FFI bridge)
      - Reuse existing run_latency_test() from run.py
      - Include sub-step timings from A5
    
    Stage 4: ROS middleware overhead
      - Compare Tier A (FFI direct) vs Tier B (ROS service) for same inputs
      - Delta = ROS serialization + transport + callback overhead
    
    Stage 5: End-to-end total (requires mock launch)
      - Reuse Tier B Method A from run_tier_b.py
    ```
  - Output: CSV with per-stage timings per object per repetition

- [ ] **B2. Create offline twist propagation benchmark** (within `run_latency_stages.py`)
  - Extract the core computation functions from `twist_propagation_node.py` into testable units:
    - `_estimate_twist()` — already a pure function on pose buffer
    - `_propagate_and_find_hit()` — uses KDTree on point cloud
    - Click cluster sampling (`_sample_spherical_shell_clicks`)
  - Create synthetic pose buffers (simulating approach trajectories) and point clouds from the 10-object dataset
  - Measure 30 repetitions per object, report mean/P95/P99
  - Rationale: This isolates the twist propagation computation from the ROS timer cycle, measuring pure algorithmic cost

- [ ] **B3. Create segmentation inference benchmark** (within `run_latency_stages.py`)
  - Call the MinkowskiEngine inference server directly via HTTP
  - Use synthetic clouds from the 10-object dataset with pre-generated click positions
  - Measure: encode_time_ms, http_roundtrip_ms, decode_time_ms, total_ms
  - 30 repetitions per object
  - Note: Requires the segmentation Docker container to be running
  - Rationale: This is the largest unknown latency contributor and cannot be measured offline without the inference server

- [ ] **B4. Add sub-step timing to the FFI bridge** (`tests/test1_software_verification/ffi_bridge.py`)
  - Extend `GraspComputeResponseFFI` ctypes struct to include new sub-timing fields from A5
  - Update `response_to_dict()` to extract sub-timings
  - Rationale: Enables the test script to report TSDF time, SMC time, scoring time separately

### Part C: Integrate with Existing Test Infrastructure

- [ ] **C1. Update `run.py` to include per-stage latency in the latency test**
  - After running the existing Tier A latency test, also run the per-stage benchmarks
  - Merge results into a combined CSV: `results/latency_per_stage_results.csv`
  - Fields: `object, repetition, stage, time_ms` (long format for easy plotting)

- [ ] **C2. Update `plot_results.py` to visualize per-stage latency**
  - Add a stacked bar chart showing the latency budget breakdown per object
  - Add a waterfall chart showing the cumulative latency from EMG to motor command
  - Update the LaTeX table to include per-stage P95 values

- [ ] **C3. Update the LaTeX results section template**
  - Add a per-stage latency breakdown table matching the test specification format
  - Include the MAR/IDE pass/fail verdicts per stage and total

### Part D: Validation and Documentation

- [ ] **D1. Run the full benchmark on the 10-object synthetic dataset**
  - Execute all stages for all 10 objects with 30 repetitions each
  - Verify that the total (sum of stages) is consistent with the Tier B end-to-end measurement
  - Check MAR ≤ 400 ms and IDE ≤ 100 ms thresholds

- [ ] **D2. Document the latency budget breakdown**
  - Create a summary table:
    ```
    | Stage | Mean (ms) | P95 (ms) | P99 (ms) | % of Total |
    |-------|-----------|----------|----------|------------|
    | Pipeline Manager | ... | ... | ... | ... |
    | Twist Propagation | ... | ... | ... | ... |
    | Segmentation | ... | ... | ... | ... |
    | Grasp Preshaping | ... | ... | ... | ... |
    | ROS Middleware | ... | ... | ... | ... |
    | TOTAL | ... | ... | ... | 100% |
    ```

## Verification Criteria

- [ ] Every pipeline stage from EMG trigger to motor command has an independent timing measurement
- [ ] Per-stage measurements are repeatable with P95 variance < 20% of mean
- [ ] The sum of per-stage measurements is within 15% of the end-to-end Tier B measurement (accounting for parallelism and measurement overhead)
- [ ] The total P95 latency is checked against MAR ≤ 400 ms and IDE ≤ 100 ms
- [ ] Results are generated for all 10 synthetic objects with ≥ 30 repetitions each
- [ ] CSV output is in a format compatible with the existing `plot_results.py` visualization pipeline

## Potential Risks and Mitigations

1. **Segmentation server unavailable for offline testing**
   Mitigation: The segmentation benchmark (B3) requires the Docker inference server. Design the script to gracefully skip this stage and mark it as "requires live server" in the output. The other stages can run fully offline.

2. **Twist propagation functions are tightly coupled to the ROS node class**
   Mitigation: The core computation functions (`_estimate_twist`, `_propagate_and_find_hit`, `_sample_spherical_shell_clicks`) are already implemented as module-level pure functions or static methods. Extract and call them directly without instantiating the full ROS node.

3. **Sub-step timing in Rust changes the FFI struct ABI**
   Mitigation: Add new timing fields at the **end** of `GraspComputeResponseFFI` to maintain backward compatibility. Existing consumers that don't read the new fields are unaffected. Version the API version number.

4. **Timing instrumentation adds overhead to the production pipeline**
   Mitigation: All timing markers use `time.perf_counter()` (Python) and `std::time::Instant::now()` (Rust), which are sub-microsecond overhead. The timing data is published on separate diagnostic topics that are not subscribed to in production. The overhead is negligible (<0.1 ms per marker).

5. **End-to-end Tier B test is flaky due to ROS timing variability**
   Mitigation: Use 30+ repetitions and report P95/P99 rather than mean. The per-stage offline measurements provide stable baselines that are not affected by ROS middleware jitter.

## Alternative Approaches

1. **ROS 2 tracing (ros2_tracing / tracetools)**: Use LTTng-based tracing to capture kernel-level timestamps for every ROS callback, subscription, and publication. This provides nanosecond-precision distributed tracing without code changes. Trade-off: Requires LTTng setup, kernel patches on Jetson, and significant tooling investment. Not practical for a one-off benchmark.

2. **Header-based timestamp propagation**: Embed a custom header with a "pipeline start" timestamp that gets passed through each node. Each node records its own receive time and forwards the header. Trade-off: Requires modifying message types or using custom headers, which breaks the standard ROS message types currently in use.

3. **External packet capture (Wireshark + DDS dissector)**: Capture DDS traffic on the loopback interface and measure inter-message latency from packet timestamps. Trade-off: Requires DDS dissector for the specific serialization format (CDR), complex setup, and doesn't capture in-process computation time.

4. **Recommended approach (this plan)**: Code-level `perf_counter` / `Instant::now()` markers at each stage boundary, published on diagnostic topics and collected by the test script. This is the simplest, most maintainable, and most accurate approach for measuring algorithmic computation time per stage.
