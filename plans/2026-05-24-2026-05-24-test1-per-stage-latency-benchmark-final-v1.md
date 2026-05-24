# Test 1: Per-Stage Latency Benchmark — Final Report

## Objective

Measure computation time from EMG signal to autonomous motor command output (Req 2.4), decomposed into individual pipeline stages. Validate against MAR ≤ 400 ms and IDE ≤ 100 ms.

## Files Created

| File | Type | Purpose |
|------|------|---------|
| `tests/test1_software_verification/run_latency_stages.py` | New (785 lines) | Per-stage latency benchmark using arrival-time tapping on 5 intermediate ROS topics |
| `tests/test1_software_verification/analyze_latency_results.py` | New (101 lines) | Results analysis post-processor |

## Files Modified

### Production Code

| File | Change |
|------|--------|
| `src/twist_propagation/twist_propagation/twist_propagation_node.py` | External twist subscription (`/hand_twist`), background warm cache (pre-computes twist + KDTree while idle), `_just_activated` fast path, 50 Hz cycle rate default, status throttle (`10 Hz`), `twist_source` logging, fixed wall-clock timestamp for external twist age check |
| `config/prosthesis_config.yaml` | Updated `cycle_delay_s: 0.02`, added `hand_twist_input_topic`, `external_twist_max_age_s`, `background_cycle_enabled` |
| `src/prosthesis_launch/launch/mock.launch.py` | Added `use_mock_emg: False` (prevents internal mock EMG from overwriting test signals), fixed config file resolution path, added static TF publisher for `camera_color_optical_frame → world` |
| `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | Added `pipeline_time_ms` and `smc_iterations` to service response message string AND new `/grasp_preshaping/pipeline_timing` String topic |

### Test Infrastructure

| File | Change |
|------|--------|
| `tests/test1_software_verification/run_tier_b.py` | Parses `pipeline_time_ms` and `smc_iterations` from service response; computes `ros_overhead_ms` |
| `tests/test1_software_verification/plot_results.py` | Added `plot_per_stage_latency()` (stacked bar chart) and `plot_per_stage_waterfall()` (cumulative waterfall chart) |

## Approach: Arrival-Time Tapping

The test subscribes to **5 intermediate topics** that act as stage boundaries in the critical path. Each callback records `time.perf_counter()`. All timestamps are from the same monotonic clock — no clock-domain issues.

```
t0: Test publishes EMG gesture
t1: /pipeline/state → SEGMENTING       → Pipeline Manager latency
t2: /segmentation/click_positive        → Twist Propagation latency
t3: /segmentation/object_cloud          → Segmentation latency
t4: /pipeline/state → PLANNING          → PM Cloud Handling latency
t5: /grasp_preshaping/grasp_type        → Grasp Preshaping latency
```

### Stage Definitions

| Stage | Delta | What It Measures |
|-------|-------|-----------------|
| Pipeline Manager | t₁ - t₀ | EMG receipt → state machine → twist activation |
| Twist Propagation | t₂ - t₁ | Collision detection cycle → click publishing |
| Segmentation | t₃ - t₂ | (Simulated: object cloud publishing; Real: HTTP inference round-trip) |
| PM Cloud Handling | t₄ - t₃ | Object cloud receipt → PLANNING state transition |
| Grasp Preshaping | t₅ - t₄ | ROS service call + Rust SMC planner |
| **Total** | **t₅ - t₀** | **Full EMG → motor command** |

## Twist Propagation Optimization

### Problem Analysis (Before)

**Twist propagation: ~160 ms mean, 66.7% of total pipeline latency**

| Component | Time | Root Cause |
|-----------|------|------------|
| Pose buffer accumulation | ~40 ms | `_on_activate` clears the pose buffer; needs ≥2 fresh poses to estimate twist |
| Polling wait | ~50 ms avg | 10 Hz timer means 0-100 ms wait for next cycle tick (avg 50 ms) |
| KDTree build | ~10-30 ms | Built on first activation cycle from 10K-30K points |
| Propagation computation | <1 ms | 100 KDTree queries × ~5 µs — negligible |
| DDS + service overhead | ~5-10 ms | ROS middleware transport |
| **Total** | **~106-131 ms** | Plus ~30 ms variability from polling alignment |

### Three Optimizations Implemented

#### 1. External Twist Subscription (saves ~40 ms)

The `odom_to_pose_relay.py` already publishes real sensor-fused twist values to `/hand_twist` from OpenVINS odometry. The node previously ignored these and re-estimated velocity from pose finite differences, requiring a pose buffer that gets cleared on activation.

**Change**: Subscribe to `/hand_twist`, use it directly when fresh (≤0.5s old), fall back to finite differences when unavailable. The node logs which method is used (`twist_source: "external"` or `"estimated"`) and includes it in the status JSON.

**Critical fix**: The external twist age check originally compared message header timestamp against ROS clock, which had a clock-domain mismatch (age reported as 17+ seconds). Fixed by using wall-clock receive time (`time.perf_counter()`) instead.

#### 2. Background Warm Cache (saves ~20-30 ms)

When inactive, the cycle callback previously did nothing. Now it runs a lightweight background cycle that maintains the twist estimate and KDTree cache. On activation, the node already has a fresh twist and cached KDTree — no cold-start penalty.

**Key addition**: A `_just_activated` flag bypasses the ≥2-pose requirement on the first active cycle, using the pre-computed twist with just 1 fresh pose.

#### 3. 50 Hz Cycle Rate (saves ~40 ms)

Changed `cycle_delay_s` from `0.1` (10 Hz) to `0.02` (50 Hz). The propagation computation per cycle is <1 ms, so CPU cost is negligible. Average polling wait drops from ~50 ms to ~10 ms. Status publication throttled to 10 Hz to avoid flooding.

### Results: Before vs After Optimization

| Metric | Before (v1) | After (optimized) | Change |
|--------|-------------|-------------------|--------|
| Twist Propagation mean | 159.9 ms | 88.7 ms | **-44.5%** |
| Twist Propagation P95 | 175.9 ms | 141.2 ms | **-19.7%** |
| Total mean | 239.7 ms | 168.1 ms | **-29.9%** |
| Total P95 | 257.5 ms | 221.7 ms | **-13.9%** |
| MAR status | PASS | PASS | maintained |

## Final Latency Budget (Optimized Config, Simulated Segmentation)

### 58 Trials Across 6 Objects (cylinder_upright, ellipsoid, small_cube, l_block, mug_with_handle, notched_box)

| Stage | Mean (ms) | P95 (ms) | % of Total |
|-------|-----------|----------|------------|
| Pipeline Manager | 1.1 | 2.1 | 0.6% |
| Twist Propagation | 88.7 | 141.2 | 52.8% |
| Segmentation (simulated) | 2.5 | 3.1 | 1.5% |
| PM Cloud Handling | 0.6 | 1.0 | 0.3% |
| Grasp Preshaping (Rust) | 75.3 | 83.5 | 44.8% |
| Preshaping (ROS overhead) | 1.8 | 2.6 | 0.8% |
| **TOTAL** | **168.1** | **221.7** | **100%** |

### Requirement Verification

| Requirement | Limit | Result | Status |
|-------------|-------|--------|--------|
| **MAR (Req 2.4)** | P95 ≤ 400 ms | P95 = **221.7 ms** | **PASS** |
| **IDE (Req 2.4)** | P95 ≤ 100 ms | P95 = **221.7 ms** | **FAIL** |

## Key Findings

### 1. Twist Propagation is No Longer the Primary Bottleneck

After optimization, the Rust grasp planner (75 ms) and twist propagation (89 ms) are comparable. The 44% reduction in twist propagation was the single largest improvement.

### 2. External Twist Dependency

Without external twist, the `_just_activated` fast path uses a pre-computed twist that may have decayed during the idle period (EMA filter converges toward zero with stale poses). This causes the collision to be missed entirely — latency balloons to **587 ms** in the worst case. External twist from `/hand_twist` is **essential for correct operation**.

### 3. IDE is Not Achievable

The Rust SMC planner alone takes ~75 ms. Even with zero overhead from all other stages, the total would be ~80 ms — only barely under the 100 ms IDE target. The current 168 ms total means IDE would require either:
- Optimizing the Rust planner (most promising target)
- Accepting IDE as aspirational
- Tightening the MAR to reflect system reality

### 4. Key Implementation Lessons

| Issue | Root Cause | Fix |
|-------|-----------|-----|
| "Gesture ignored: confidence 0.00" | Internal mock EMG timer overwritten test's Float32 confidence with 0.0 | `use_mock_emg: False` in mock launch |
| No events recorded | Subscriber type mismatches: `Int32` for `PointStamped`, `String` for `Int32` | Fixed topic types |
| Pipeline stuck in SEGMENTING | Simulated segmentation published to wrong topic | Fixed: `/segmentation/input_cloud` → `/segmentation/object_cloud` |
| Pipeline stuck in SEGMENTING (sim seg) | Object cloud never published when simulating | Added direct publishing in `_on_click_positive` |
| External twist never received | Clock-domain mismatch: message timestamp vs ROS clock | Switched to wall-clock receive time |
| No collision detected | Activation delay longer than expected; hand passed the object | Increased approach distance; continuous twist publishing |
| "No points in ROI" | Preshaping ROI uses hand pose; hand far from object | Added corrected-hand-pose publishing on click |

## Remaining Work

### Critical: Real Segmentation Benchmark

- [ ] The CUDA segmentation backend is running (`make segmentation-cuda`) but not yet integrated into the latency test
- [ ] Quick diagnostic showed it returns "no foreground points" when click point doesn't land on actual object
- [ ] Expected latency: **50-200 ms** (HTTP POST + MinkowskiEngine inference)
- [ ] This could push total latency past the 400 ms MAR if the real system also has click-to-object misalignment
- [ ] **Priority fix**: Ensure the click point lands on the actual object cloud, not on the offset-propagated area

### Further Optimization: Rust Planner

- [ ] At **75 ms mean**, the Rust SMC sampler is now the dominant stage (44.8%)
- [ ] Potential improvements: reduce sample count, simplify TSDF construction, use GPU acceleration
- [ ] A 50% reduction (~37 ms) would bring total to ~130 ms, approaching IDE

### Numerical Simulation Verification

- [ ] The SDF-based numerical simulation check (`can_it_grasp_numerical_thread`) needs to be validated against the analytical SDF (`can_it_grasp_analytical`)
- [ ] Run Tier A tests with `simulate_segmentation` to compare analytical vs numerical results for all 10 objects

## How to Run

```bash
# Terminal 1: Start mock system (including segmentation bridge)
ros2 launch prosthesis_launch mock.launch.py headless:=True

# Terminal 2: Run per-stage latency benchmark with simulated segmentation
python3 tests/test1_software_verification/run_latency_stages.py \
    --objects cylinder_upright ellipsoid small_cube \
    --repetitions 10 \
    --simulate-segmentation

# Terminal 3: Analyze results
python3 tests/test1_software_verification/analyze_latency_results.py
```

### Available Flags

| Flag | Purpose |
|------|---------|
| `--objects` | Space-separated list of object names (default: all parametric objects) |
| `--repetitions` | Trials per object (default: 30) |
| `--simulate-segmentation` | Bypass real CUDA inference server (publish object cloud directly) |
| `--no-external-twist` | Disable external twist subscription for A/B comparison |
| `--label` | Label for results file (e.g., "optimized", "baseline") |
