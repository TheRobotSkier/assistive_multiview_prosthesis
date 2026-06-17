# Rate Decline Puzzle — Root Cause Analysis

## Objective

Determine why the GTSAM output rate at `gtsam_tracker_node` consistently declines from ~15 Hz (peak) to ~8.7 Hz (last-interval) over a 170-second bag replay, with the decline magnitude being **independent of graph size** (same end rate for ~168 variables with marginalization vs ~4104 variables without).

## Key Observations from the Data

### Convergence Evidence

| Metric | Iteration 17 (marg ON, 168 vars) | Iteration 18 (marg OFF, 4104 vars) | Delta |
|--------|-------|-------|-------|
| Bag duration | 170.65 s | 171.53 s | — |
| GTSAM avg Hz | 11.68 | 11.81 | +1.1% (slight improvement) |
| GTSAM peak Hz | 15.1 | 15.1 | **identical** |
| **GTSAM last Hz** | **8.6** | **8.7** | **within noise** |
| GTSAM mid Hz | 11.6 | 12.3 | +6% (no-marg faster mid-run) |
| TF Hz | 45.36 | 45.63 | within noise |
| CPU avg/max | 63.9% / 99.2% | 63.2% / 91.6% | slightly better no-marg |
| Odom head mid→last | 118.2 → 142.4 | 122.9 → 142.4 | **identical end rate** |
| Odom arm mid→last | 120.4 → 148.7 | 120.8 → 148.7 | **identical end rate** |
| Total GTSAM msgs | 1993 | 2026 | +1.7% |

### Key Inference

Both configurations converge to the **same last-interval rate (~8.7 Hz)** despite a **24× difference in graph variable count**. This means the bottleneck is **NOT** in:
- ISAM2 forward-backward pass scaling (O(tree depth))
- Relinearization cost scaling (O(variable count))
- Linear-approximation factor accumulation from marginalization
- Any other graph-size-dependent operation

The bottleneck must be in something that:
1. Has the **same time-per-cycle cost profile** for both graph sizes
2. **Grows progressively** over the run duration (~170 s)
3. Is **not mitigated** by disabling marginalization

## Root Cause Candidates (ranked by likelihood)

### Candidate 1: GTSAM ISAM2 Internal Factor Graph Accumulation (HIGH)

**Mechanism**: Each `ISAM2::update()` call copies the new factors (2-3 per cycle) and values into the ISAM2 object's internal `NonlinearFactorGraph` and `Values` structures. These accumulate for the entire run duration regardless of marginalization state. After ~2040 cycles:
- ~4000-6000 factors stored internally (2 per cycle × 2040 + range factors)
- ~4104 variable values stored internally
- ISAM2 maintains a `VariableIndex` that maps each variable key to its connected factor indices — this structure grows linearly with total factors ever added.

**Why it's graph-size-independent**: In BOTH configurations, the same number of factors are added per cycle (2 between-factors + 1 range factor every 5th). Marginalization removes OLD variable entries from the Bayes tree but does NOT remove factors from ISAM2's internal factor graph — it only adds new linear-approximation factors. So the internal factor graph grows at the same rate in both configurations.

**Why it slows progressively**: The `VariableIndex` and associated internal bookkeeping structures grow linearly with cycles. Each ISAM2 `update()` must:
1. Add new factors to the internal factor graph (updates the index — O(1) but the index grows)
2. Compute which existing variables are connected to new factors (lookup in the growing index)
3. During the marginalization-eligibility or relinearization-eligibility checks: traverse or iterate over the accumulated structure

**Predicted fix**: Periodically purge the ISAM2 internal factor graph of old, already-marginalized factors. Or set a `ISAM2Params::cacheLinearizedFactors` option to limit internal storage.

**Evidence strength**: This is the strongest candidate because it predicts the exact observed behavior: same slowdown regardless of marginalization state, progressive over time, and not fixable by reducing graph size.

### Candidate 2: DDS TF Topic Backpressure from Overloaded Subscribers (HIGH)

**Mechanism**: The `_publish_pose()` method calls `self._tf_broadcaster.sendTransform(tf_msg)` inside the timer callback (2 calls per cycle). The `TransformBroadcaster` publishes to the `/tf` topic. Several subscribers show clear signs of overload:
- `tf_pipeline_diagnostics`: 2006+ WARN messages per run
- `pipeline_diagnostics_node`: ~1996 WARN messages
- `openvins_odom_tf_relay`: ~2078 WARN messages

If the `/tf` topic publisher uses RELIABLE QoS (default for `tf2_ros.TransformBroadcaster` in ROS2), a slow subscriber creates DDS backpressure. As the run progresses and subscriber buffers fill, `sendTransform()` latency increases progressively.

With GTSAM TF broadcast at ~24 Hz (2 transforms × ~12 Hz) plus other TF broadcasters in the system (openvins_odom_tf_relay, openvins_realsense_tf_bridge_node, camera_mount_tf_publisher), the total `/tf` rate of ~45 Hz matches the observed effective rate. Any of the TF subscribers that accumulates state or slows down over time would cause progressively increasing latency.

**Why it's graph-size-independent**: The TF broadcast rate and subscriber load are entirely independent of the GTSAM graph size. Both iteration 17 and 18 have the same `/tf` rate (~45 Hz).

**Predicted fix**: Change the TF broadcaster's QoS profile to BEST_EFFORT, or reduce TF broadcast frequency, or diagnose the overloaded TF subscribers.

### Candidate 3: Python Object Accumulation and GC Pressure (MEDIUM)

**Mechanism**: Each `_graph_update()` cycle creates ~5-10 Python wrapper objects for GTSAM C++ types:
- 2× `gtsam.Pose3` (from numpy arrays)
- 2× `gtsam.BetweenFactorPose3` (odometry factors)
- 1× optional `gtsam.BetweenFactorPose3` (range factor)
- 2× `gtsam.noiseModel.Diagonal` (or reused)
- `gtsam.NonlinearFactorGraph` and `gtsam.Values` replacement after each update
- 2× `PoseWithCovarianceStamped`, 2× `TransformStamped`, 1× `TFMessage` per publish cycle
- Numerous numpy arrays (`np.eye(4)`, temporary arrays in `pose_to_matrix`, etc.)

Over 2040 cycles, this creates ~10,000-20,000 Python objects. While most are short-lived and collected by reference counting, the cyclic garbage collector (`gc`) may need to scan an increasingly large object graph. Python's GC overhead scales with the number of live objects, and if any reference cycles exist (e.g., through GTSAM's shared_ptr management), these accumulate and require full GC sweeps.

**Why it's graph-size-independent**: Object creation per cycle is the same regardless of graph size — the graph management happens in C++, not Python.

**Predicted fix**: Explicitly invoke `gc.collect()` periodically. Or reduce object creation by reusing GTSAM noise models and pose objects where possible.

### Candidate 4: ISAM2 Relinearization Threshold Causing Full-Tree Rebuild (MEDIUM-LOW)

**Mechanism**: With `relinearizeThreshold=0.01` and `relinearizeSkip=5`, every 5th cycle ISAM2 checks ALL variables for relinearization. Each variable accumulates ~5 cm of error between checks, which exceeds the 0.01 threshold. This marks ALL variables for relinearization, triggering a full or partial Bayes tree rebuild.

While the re-elimination cost for a chain is O(N), the check itself iterates over all variables. For 168 variables, the check is trivial. For 4104 variables, the check is still fast in C++ (~microseconds per variable). Neither should cost the ~50 ms of extra per-cycle time.

**Why it's graph-size-independent despite different N**: The relinearization check time (O(N)) differs by 24× between configurations. But if both are <1 ms, the difference is negligible. The observed extra 50 ms must come from elsewhere.

**Evidence strength**: Low — can't explain the magnitude of the slowdown or the convergence to the same end rate.

### Candidate 5: ROS2 Timer Scheduling with Subscription Flooding (MEDIUM)

**Mechanism**: In the `SingleThreadedExecutor`, when the timer callback finishes, the executor processes ALL pending subscription callbacks before checking the timer. With odom at ~150 Hz each (~300 combined msgs/s), ~20-30 odom callbacks queue up during each ~115 ms timer callback. Processing these adds ~20-30 µs — negligible.

However, there is a more subtle effect: as the timer callback takes longer, more odom messages queue up, creating a longer subscription-processing tail, which in turn delays the next timer callback. This creates a positive feedback loop where any initial slowdown is amplified.

**Why it's graph-size-independent**: The feedback loop amplifies whatever baseline slowdown exists, but the initial slowdown must come from another source (candidates 1-3).

**Evidence strength**: Amplifier, not root cause. The base rate decline requires explanation from candidates 1-3.

## Empirical Validation Plan

To confirm the root cause, instrument the `_graph_update()` callback with per-operation timing:

- [ ] **Operation 1**: Time `_graph.update(marginalize_keys=[])` in isolation (surround with `time.perf_counter()` calls, log each cycle's duration)
- [ ] **Operation 2**: Time `_publish_poses()` in isolation (the full publish path including TF broadcast)
- [ ] **Operation 3**: Time `add_odometry_factor()` in isolation
- [ ] **Operation 4**: Measure cumulative Python object count via `len(gc.get_objects())` at 10 s intervals
- [ ] **Operation 5**: Measure GTSAM ISAM2 internal factor count via `self._graph._isam.getFactorsUnsafe().size()` (or similar introspection) at 10 s intervals
- [ ] **Operation 6**: Time TF broadcast with a dedicated measurement — call `sendTransform` in isolation with a no-op subscriber to measure baseline vs. with real subscribers

If Candidate 1 is correct:
- Operation 1 will show the ISAM2 `update()` duration increasing from ~5 ms to ~55 ms over the run
- Operation 5 will show the internal factor count growing linearly from ~3 to ~6000
- Operation 2 will show constant time for publish (excluding TF)

If Candidate 2 is correct:
- Operation 2 will show the TF broadcast time increasing from ~0.5 ms to ~50 ms over the run
- Operations 1 and 5 will show constant time

If Candidate 3 is correct:
- Operation 4 will show the Python object count growing linearly and GC pauses increasing
- All GTSAM operations will show slight increases from memory pressure

## Diagnostic Probe Strategy

### Probe A: ISAM2 Internal Factor Count

Add a diagnostic method to `TrajectoryFactorGraph`:

```python
@property
def internal_factor_count(self) -> int:
    """Number of factors stored inside ISAM2's internal factor graph."""
    return self._isam.getFactorsUnsafe().size()
```

Log this value every 10 s alongside the existing stats.

### Probe B: Per-Operation Timing

Add `time.perf_counter()` measurements around each major section of `_graph_update()`:

```python
t0 = time.perf_counter()
self._graph.add_odometry_factor(...)
t1 = time.perf_counter()
self._graph.update(marginalize_keys=[])
t2 = time.perf_counter()
self._publish_poses()
t3 = time.perf_counter()
```

Log max/avg duration every 10 s so the data volume stays manageable.

### Probe C: TF Broadcast Latency Measurement

Create a test harness that isolates `TransformBroadcaster.sendTransform()` to measure its baseline latency and check for progressive increase:

```python
def _benchmark_tf_broadcast(self, n=1000):
    """Time n consecutive TF broadcasts."""
    import time
    tf_msg = TransformStamped()
    # ... fill msg ...
    t0 = time.perf_counter()
    for _ in range(n):
        self._tf_broadcaster.sendTransform(tf_msg)
    t1 = time.perf_counter()
    return (t1 - t0) / n
```

### Probe D: Python GC Object Count

```python
import gc
gc.collect()
live_count = len(gc.get_objects())
```

Log this every 10 s in `_log_stats()`.

## Recommendations

### Short-term (immediate investigation)

1. **Add per-operation timing probes** (Probe B) to the next iteration's `_graph_update()` to isolate which section of the callback accounts for the ~49 ms of extra time per cycle by run end. This is the single most important measurement — it will directly identify whether the bottleneck is in ISAM2::update(), TF broadcast, or Python overhead.

2. **Log ISAM2 internal factor count** (Probe A) to verify the factor accumulation hypothesis.

### Medium-term (if ISAM2 internal accumulation is confirmed)

3. **Set `ISAM2Params::setCacheLinearizedFactors(false)`** — if this GTSAM option exists, it may limit internal factor storage.

4. **Re-enable marginalization but with a more aggressive scheme**: Marginalize NOT just by timestamp but also by factor count. Periodically purge old nonlinear factors from ISAM2's internal graph while keeping the Bayes tree clean.

5. **Alternative: Reset ISAM2 periodically** (every 30-60 s) by re-seeding from the current estimate, similar to the existing `_reset_graph()` but triggered on a timer rather than only on failure.

### Medium-term (if TF backpressure is confirmed)

6. **Change TF broadcaster QoS to BEST_EFFORT** to eliminate backpressure.

7. **Reduce TF broadcast frequency** — broadcast only every Nth cycle instead of every cycle.

8. **Diagnose and fix the overloaded TF subscribers** (`tf_pipeline_diagnostics`, `pipeline_diagnostics_node`, `openvins_odom_tf_relay`) that generate thousands of WARN messages.

## Verification Criteria

- [ ] Per-operation timing shows which section of `_graph_update()` grows by ~49 ms over 170 s
- [ ] ISAM2 internal factor count logged and confirmed to grow linearly with cycle count
- [ ] TF broadcast latency measured and confirmed stable or growing
- [ ] Python GC object count measured and confirmed stable or growing
- [ ] After applying the confirmed fix: GTSAM rate stabilizes at ≥13.5 Hz with <5% mid-to-last decline
