# Per-Stage Latency Benchmark for Test 1 Software Verification (v2)

## Objective

Design a **non-invasive, toggleable** per-stage latency benchmark that records wall-clock arrival times of messages on intermediate pipeline topics. This decomposes the EMG-to-motor-command path into measurable segments without modifying any production pipeline code.

Satisfies **Requirement 2.4**: "Maximum computation time from EMG signal to autonomous motor command output" — MAR ≤ 400 ms, IDE ≤ 100 ms.

## Approach: Arrival-Time Tapping

Instead of adding timing instrumentation to pipeline nodes, the test script subscribes to all intermediate topics and records `time.perf_counter()` when each message arrives. This gives us:

- **Zero changes to production code** — all measurement happens in the test script
- **Fully toggleable** — only runs when this specific test is invoked
- **Real wall-clock latency** — captures exactly what the spec asks for, including ROS middleware overhead
- **Per-stage breakdown** — by differencing arrival times between consecutive topics

### Why Not Header-Stamp Comparison?

Many intermediate topics use `std_msgs/Int32`, `Float64`, `String`, `Float64MultiArray` — these have **no `header.stamp` field**. The topics that do have headers (`PointCloud2`, `PoseStamped`, `TwistStamped`, `PointStamped`) use different clock domains (some from `get_clock().now()`, some pass-through from sensors). Arrival-time recording avoids all clock-domain issues.

### Pipeline Critical Path and Observable Topics

```
Test publishes EMG gesture ───────────────────────────────────────────── t₀
  │
  ▼
/pipeline/state changes to SEGMENTING (1) ───────────────────────────── t₁
  │   [Stage A: Pipeline Manager overhead = t₁ - t₀]
  │
  ▼
/segmentation/click_positive arrives ─────────────────────────────────── t₂
  │   [Stage B: Twist propagation collision + click = t₂ - t₁]
  │
  ▼
/segmentation/object_cloud arrives ───────────────────────────────────── t₃
  │   [Stage C: Segmentation inference = t₃ - t₂]
  │
  ▼
/pipeline/state changes to PLANNING (2) ─────────────────────────────── t₄
  │   [Stage D: Pipeline manager object cloud handling = t₄ - t₃]
  │
  ▼
/grasp_preshaping/grasp_type arrives ────────────────────────────────── t₅
  │   [Stage E: Grasp preshaping (Rust) + ROS service overhead = t₅ - t₄]
  │
  ▼
/grasp_preshaping/target_finger_closures arrives ────────────────────── t₅'
  │   [Published almost simultaneously with grasp_type]
  │
  ──────────────────────────────────────────────────────────────────────
  TOTAL = t₅ - t₀
```

Note: The test publishes cloud + pose + twist data **before** the EMG trigger, so the preshaping bridge already has valid inputs when the service is called. The measured latency is purely the EMG-trigger-to-command path.

## Implementation Plan

### Part A: Create the Per-Stage Latency Test Script

- [ ] **A1. Create `tests/test1_software_verification/run_latency_stages.py`**
  - New standalone script (like `run_tier_b.py`) that subscribes to intermediate topics and records arrival times
  - Requires the mock launch running: `ros2 launch prosthesis_launch mock.launch.py`
  - Usage: `python run_latency_stages.py [--objects ...] [--repetitions 30]`
  - Rationale: This is the core deliverable — a self-contained benchmark that produces per-stage latency data

- [ ] **A2. Implement multi-topic subscriber with arrival-time recording**
  - Subscribe to these topics with callbacks that record `time.perf_counter()`:
    | Topic | Message Type | What It Signals |
    |-------|-------------|-----------------|
    | `/pipeline/state` | `Int32` | Pipeline manager state transitions (IDLE=0, SEGMENTING=1, PLANNING=2, APPROACHING=3) |
    | `/segmentation/click_positive` | `PointStamped` | Twist propagation detected collision and published clicks |
    | `/segmentation/object_cloud` | `PointCloud2` | Segmentation inference completed |
    | `/grasp_preshaping/grasp_type` | `Int32` | Grasp planner produced a result |
    | `/grasp_preshaping/target_finger_closures` | `Float64MultiArray` | Motor command output |
  - Each callback stores `(topic, perf_counter_time, message_data)` into a per-trial event list
  - Use a `threading.Lock` to protect the event list (same pattern as `run_tier_b.py:134-146`)
  - Rationale: These 5 topics cover every stage boundary in the critical path. The arrival-time deltas between consecutive topics give per-stage latency.

- [ ] **A3. Implement the trial execution loop**
  - For each object × repetition:
    1. **Clear** previous trial events
    2. **Publish** synthetic cloud + pose + twist (pre-populate the preshaping bridge inputs)
    3. **Wait** 100ms for data to propagate (same as `run_tier_b.py:219`)
    4. **Record** `t_start = time.perf_counter()` 
    5. **Publish** EMG POWER gesture (`Int32(data=1)`) on `/emg/gesture_label`
    6. **Wait** for `/grasp_preshaping/grasp_type` to arrive (with 15s timeout)
    7. **Record** all event timestamps from the trial
    8. **Publish** EMG OPEN gesture (`Int32(data=3)`) to reset pipeline to IDLE
    9. **Wait** 500ms for pipeline reset (same as `run_tier_b.py:268`)
  - Rationale: This mirrors the existing Tier B Method A pattern but adds intermediate topic tapping

- [ ] **A4. Compute per-stage latencies from arrival times**
  - For each trial, extract these stage durations:
    | Stage | Computation | Description |
    |-------|------------|-------------|
    | `pipeline_manager` | `t₁ - t₀` | EMG receipt → state transition to SEGMENTING |
    | `twist_propagation` | `t₂ - t₁` | State transition → click published |
    | `segmentation` | `t₃ - t₂` | Click received → segmented cloud published |
    | `pm_to_preshaping` | `t₅ - t₄` | PLANNING state → grasp_type published |
    | `total` | `t₅ - t₀` | EMG trigger → grasp command |
  - Where `t₀` = EMG publish time, `t₁` = first `/pipeline/state` change to 1 (SEGMENTING), `t₂` = first `/segmentation/click_positive`, `t₃` = first `/segmentation/object_cloud`, `t₄` = `/pipeline/state` change to 2 (PLANNING), `t₅` = `/grasp_preshaping/grasp_type`
  - Handle edge cases: if a topic message doesn't arrive within the trial, record `NaN` for dependent stages
  - Rationale: This decomposition identifies which stage dominates the latency budget

- [ ] **A5. Handle the twist propagation path correctly**
  - In the mock launch, the twist propagation node starts **inactive**. The pipeline manager activates it on EMG trigger. The first collision detection cycle may not fire immediately because:
    - The node needs pose data (from mock cloud publisher or test-injected)
    - The speed gate requires `min_twist_linear_mps > 0.02`
    - The cycle runs at `cycle_delay_s = 0.1s` intervals
  - Two options for ensuring the twist propagation fires:
    - **Option 1 (preferred)**: The test also publishes to `/hand_pose` with an approach trajectory that has sufficient velocity, so twist propagation naturally detects a collision
    - **Option 2 (fallback)**: If the mock launch already has a `mock_cloud_publisher` that provides trajectory data, rely on that
  - Verify by checking what `mock_cloud_publisher` provides (it may already publish moving hand poses)
  - Rationale: If twist propagation never fires, the pipeline manager's alternative path (directly calling preshaping on object cloud receipt) would be measured instead — which is a valid but different measurement

- [ ] **A6. Output results to CSV**
  - Write to `results/latency_per_stage_results.csv`
  - Fields: `object, repetition, pipeline_manager_ms, twist_propagation_ms, segmentation_ms, pm_to_preshaping_ms, total_ms, path_taken, status`
  - `path_taken` indicates whether the trial went through twist propagation or the direct pipeline manager path
  - Also write a summary CSV: `results/latency_per_stage_summary.csv` with mean/P95/P99 per object per stage
  - Rationale: Long-format per-trial CSV enables detailed analysis; summary CSV enables quick comparison

### Part B: Add `pipeline_time_ms` to the Service Response (Minimal Change)

- [ ] **B1. Append `pipeline_time_ms` to the Trigger response message**
  - In `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:503-512`, add one line to the response message string:
    - Append `", pipeline_time_ms=" + std::to_string(ffi_response.pipeline_time_ms)`
  - This is a **1-line change** that makes the Rust algorithmic time available to any service caller
  - The test script can then parse `pipeline_time_ms=XX` from the response to separate Rust compute time from ROS overhead
  - Rationale: This tiny change gives us the ability to decompose Stage E (`pm_to_preshaping`) into "ROS service overhead" vs "Rust algorithmic time" — without it, we can't distinguish the two. This is the only production code change in the entire plan.

### Part C: Integrate with Existing Test Infrastructure

- [ ] **C1. Update `plot_results.py` to visualize per-stage latency**
  - Add a stacked bar chart: each bar = one object, stacked segments = per-stage latency
  - Add a horizontal line at 400 ms (MAR threshold)
  - Add a waterfall/cascade chart showing cumulative latency from EMG to command
  - Rationale: Visual breakdown makes it immediately clear which stage dominates

- [ ] **C2. Update `run_tier_b.py` to also capture `pipeline_time_ms` from service response**
  - In Method B, parse the `pipeline_time_ms=XX` from the response message (once B1 is done)
  - Add `pipeline_time_ms` as a column in the Tier B output CSV
  - Compute `ros_overhead_ms = total_latency_ms - pipeline_time_ms`
  - Rationale: This enriches the existing Tier B data with zero additional test runs

- [ ] **C3. Generate LaTeX table for the report**
  - Produce a table matching the test specification format:
    ```
    | Stage | Mean (ms) | P95 (ms) | P99 (ms) | % of Total |
    ```
  - Include MAR/IDE pass/fail verdicts

### Part D: Validation

- [ ] **D1. Verify the mock launch produces the expected topic sequence**
  - Run the mock launch, inject an EMG trigger, and confirm that all 5 intermediate topics fire in the expected order
  - If the mock launch doesn't have a moving hand pose source, the test may need to publish approach trajectories to `/hand_pose`
  - Check what `mock_cloud_publisher` (`src/pipeline_manager/pipeline_manager/mock_cloud_publisher.py`) publishes

- [ ] **D2. Run the full benchmark on the 10-object synthetic dataset**
  - 30 repetitions per object
  - Verify total latency is consistent with existing Tier B measurements
  - Verify the sum of per-stage times approximately equals the total (within 15%, accounting for parallelism and measurement overhead)

- [ ] **D3. Cross-validate with existing data**
  - Compare `pm_to_preshaping` stage time against the existing Tier A `pipeline_time_ms` data
  - The difference should approximate ROS service overhead (serialization + DDS transport)
  - Compare total latency against the existing Tier B service-method measurements

## Verification Criteria

- [ ] The test script produces per-stage latency for all pipeline stages from EMG trigger to grasp command
- [ ] No production pipeline code is modified (except the 1-line `pipeline_time_ms` addition to the service response)
- [ ] The test is fully toggleable — it only runs when explicitly invoked, does not affect other tests
- [ ] Results are generated for all 10 synthetic objects with ≥ 30 repetitions each
- [ ] Per-stage P95 values are reported and checked against the 400 ms MAR budget
- [ ] The total P95 latency matches existing Tier B measurements within 15%
- [ ] CSV output is compatible with the existing `plot_results.py` visualization pipeline

## Potential Risks and Mitigations

1. **Twist propagation may not fire in the mock launch**
   Mitigation: Investigate `mock_cloud_publisher` to understand what data it provides. If it doesn't publish moving hand poses, the test script will publish approach trajectories to `/hand_pose` with sufficient velocity to trigger collision detection. If twist propagation fundamentally cannot fire in the mock setup, the test will measure the alternative path (pipeline manager directly calling preshaping) and document this as a limitation.

2. **Segmentation inference server not running**
   Mitigation: The segmentation Docker container must be running for the full pipeline test. The script should check for the inference server at startup (`http://127.0.0.1:5678/health`) and provide a clear error message if unavailable. If segmentation cannot run, the test degrades gracefully by measuring only the stages that work and marking others as "requires inference server."

3. **Topic ordering is non-deterministic**
   Mitigation: The arrival times are recorded in a single-threaded callback context within the test node. ROS 2 callbacks for a single node are serialized by the executor, so the ordering is deterministic within the test's perspective. Use message sequence numbers or content matching (e.g., pipeline state value) to disambiguate.

4. **The 150 ms preshaping call delay inflates Stage D**
   Mitigation: This is a real, configurable delay (`preshaping_call_delay_s = 0.15` in the twist propagation config). It should be measured and reported as part of the pipeline latency — it's a deliberate design choice, not measurement error. The per-stage breakdown will make this visible.

5. **Clock skew between test node and pipeline nodes**
   Mitigation: All nodes run on the same host (or same Docker container). `time.perf_counter()` is monotonic within a process. Since the test node records all arrival times using its own clock, there is no cross-process clock skew — all deltas are measured on the same clock.

## Alternative Approaches Considered

1. **Header-stamp comparison**: Compare `header.stamp` values across topics. Rejected because many intermediate topics (`Int32`, `Float64`, `String`) have no header, and stamped topics use different clock domains (sensor clock vs. host clock).

2. **Code-level `perf_counter` markers in each node**: Add timing instrumentation to every pipeline node. Rejected because it modifies production code, is not toggleable, and adds logging noise during normal operation.

3. **ROS 2 tracing (LTTng)**: Kernel-level distributed tracing. Rejected due to setup complexity, kernel requirements on Jetson, and significant tooling investment for a one-off benchmark.

4. **Arrival-time tapping (chosen)**: Subscribe to intermediate topics from the test script, record `time.perf_counter()` at each arrival. Zero production code changes, fully toggleable, measures real wall-clock latency including ROS overhead. The only production change is the optional 1-line addition of `pipeline_time_ms` to the service response for finer decomposition.
