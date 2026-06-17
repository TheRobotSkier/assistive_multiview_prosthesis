# Iteration 4 — 2026-06-17

## Goal

Fix the ISAM2 fixed-lag marginalization that was silently a no-op, causing the
factor graph to grow unboundedly over time. This directly limits the GTSAM
output rate (12.1 Hz vs 15 Hz nominal) because each ISAM2 update slows down
as the graph accumulates thousands of variables.

## What was investigated

### Replay results (replay_20260617_022657 — after iteration 3)

| Topic | Count | Eff Hz | Nom Hz | Health |
|---|---|---|---|---|
| `/gtsam/arm_pose` | 2068 | 12.1 | 15 | OK |
| `/gtsam/head_pose` | 2068 | 12.1 | 15 | OK |
| `/vis/head_arm_pose` | 1 | 0.006 | 30 | STARVED |
| `/tf` | 5253 | 30.8 | 50 | OK |

**Key observations:**

1. **GTSAM output is stuck at ~12.1 Hz** — the 15 Hz timer fires every 67 ms,
   but the `_graph_update` callback takes ~82 ms per cycle, so ~19% of timer
   firings are skipped. The VIO odometry delivers at ~114 Hz, so the bottleneck
   is entirely inside the GTSAM processing pipeline.

2. **No trajectory ATE/RPE metrics** — the bag analysis does not compute deep
   replay (mcap/mcap-ros2-support is not installed), so there are no odom→gtsam
   agreement metrics. The only visible metric is topic effective Hz.

3. **No errors, crashes, or TF jumps** — the pipeline runs cleanly. The
   bottleneck is computational, not a failure mode.

### Comparison across iterations

| Metric | Iter1 (021016) | Iter2 (021726) | Iter3 (022657) |
|---|---|---|---|
| `/gtsam/head_pose` Hz | 11.93 | 11.9 | 12.135 |
| `/gtsam/arm_pose` Hz | 11.93 | 11.9 | 12.135 |
| `/vis/head_arm_pose` Hz | 0.006 | 0.006 | 0.006 |
| CPU avg | 48% | — | 60% |
| CPU max | 88% | — | 82% |

The iteration-3 fix (range factor every update) gave a small improvement from
11.93 → 12.135 Hz (+0.2 Hz, +1.7%).

### Root cause: `marginalize_old_keys` was a no-op

**Location:** `src/gtsam_tracker/gtsam_tracker/factor_graph.py:350-370` (pre-fix)

The `marginalize_old_keys` method identified keys older than `lag_s` (15 s)
but **never actually told ISAM2 to marginalize them**. The implementation only
removed the keys from the internal `_key_timestamps` tracking dictionary:

```python
# PRE-FIX (ineffective "marginalization"):
for k in to_remove:
    self._key_timestamps.pop(k, None)
return len(to_remove)
```

ISAM2's `update()` was called without the `marginalizeTheta` argument, so old
variables accumulated in the Bayes tree forever:

```python
# PRE-FIX: graph grows unbounded
self._isam.update(self._new_factors, self._new_values)  # ← no marginalization!
```

**Impact over a 170 s replay:**
- ~2068 odometry factors per chain = 4136 between-factors
- ~2068 kinematic range factors (one per update after iteration 3)
- ~4000 variables in the ISAM2 graph (head + arm chains × 2000 keys)
- Each ISAM2 `update()` searches through and relinearizes this growing set

The fixed-lag parameter `lag_s = 15.0` means the window should be only
~180 keys per chain (15 s × 12 Hz), but without actual marginalization the
graph size was 11× larger than necessary (4000 vs ~360).

**Why it happens:** When ISAM2's graph grows, the `calculateEstimatePose3()`
calls (one per key lookup) become linear searches through an expanding result
set. The `update()` relinearization checks iterate over all ~4000 variables
instead of ~360. Both effects compound, causing the per-cycle time to increase
over the replay duration rather than staying constant.

## Changes made

### 1. `src/gtsam_tracker/gtsam_tracker/factor_graph.py:335-356`

**`update()`** now accepts a `marginalize_keys` parameter and passes it to
ISAM2 as the `marginalizeTheta` argument:

```python
# NEW: marginalization enabled
def update(self, marginalize_keys: Optional[List[int]] = None) -> None:
    if self._new_factors.size() == 0 and not marginalize_keys:
        return

    theta = gtsam.KeyVector()
    if marginalize_keys:
        for k in marginalize_keys:
            theta.push_back(k)

    self._isam.update(self._new_factors, self._new_values, theta)
    ...
```

This is backward-compatible — when called without arguments (as in unit tests),
`theta` is an empty `KeyVector` and ISAM2 behaves exactly as before.

### 2. `src/gtsam_tracker/gtsam_tracker/factor_graph.py:366-401`

**`marginalize_old_keys()`** now returns the list of integer keys eligible for
marginalization (instead of just a count), and also cleans up `_initialized`
and `_all_keys` tracking:

```python
# NEW: returns actual keys + full cleanup
def marginalize_old_keys(self, current_stamp: float) -> List[int]:
    threshold = current_stamp - self._lag_s
    to_remove = [k for k, t in self._key_timestamps.items() if t < threshold]
    if not to_remove:
        return []

    for k in to_remove:
        self._key_timestamps.pop(k, None)
        self._all_keys.discard(k)
        self._initialized.discard(k)
    return to_remove
```

The `_initialized.discard(k)` ensures that if a key index is ever reused after
a graph reset, the fresh initial estimate is computed from the current odometry
rather than a stale cached pose.

### 3. `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:556-583`

**Reordered the update sequence** so marginalization happens BEFORE the ISAM2
update, and the marginalized keys are passed into the same call as the new
factors:

```python
# NEW: marginalize → update (keys passed together)
old_keys: list = []
try:
    old_keys = self._graph.marginalize_old_keys(stamp)
except Exception:
    pass

try:
    self._graph.update(marginalize_keys=old_keys)
    self._consecutive_failures = 0
except Exception as exc:
    ...
```

Previously the calls were in the opposite order (update first, then
marginalize) and the marginalize call was dead code. The old redundant
`marginalize_old_keys` call after the update has been removed.

## Expected impact

**Localization precision (goal 1):**

1. **Higher GTSAM output rate** — With the graph bounded at ~360 variables
   (180 head + 180 arm for 15 s lag), each ISAM2 update will be significantly
   faster. We expect the output rate to rise from 12.1 Hz toward the 15 Hz
   timer rate. The improvement will be most visible in the latter half of the
   replay, where the un-fixed graph would have been largest.

2. **More frequent pose estimates** — More poses per second means the
   smoothed trajectory tracks the true motion more closely, reducing
   interpolation error in downstream consumers.

3. **Consistent per-update latency** — With a bounded graph, the update time
   stays constant throughout the replay instead of increasing. This prevents
   late-cycle timing issues.

4. **Proper fixed-lag behaviour** — The linear marginalization factors that
   ISAM2 creates when removing old variables properly preserve the information
   from marginalized poses rather than discarding it. This is the theoretically
   correct approach to maintaining a fixed-lag window.

**TSDF fusion (goal 2, indirect):**

- At ~15 Hz pose output (vs ~12 Hz), TSDF receives ~20% more pose updates for
  pointcloud integration. More integration steps → denser fused volume with
  fewer holes and better spatial consistency.

- Tighter pose timing also reduces the temporal misalignment between pointcloud
  capture and its associated pose, which is a common source of outliers.

### Quantitative expectation

- `/gtsam/head_pose` and `/gtsam/arm_pose` effective Hz should increase from
  **~12.1 Hz to ~14–15 Hz** (approaching the 15 Hz timer rate).
- No change expected in `/vis/head_arm_pose` (golden bag limitation persists).
- CPU usage may decrease slightly as ISAM2 does less work per update.

## What to check in the next iteration

1. **GTSAM output rate** — check `bag_analysis.txt` for `/gtsam/head_pose` and
   `/gtsam/arm_pose` effective_hz. Expect 14–15 Hz (vs 12.1 Hz previously).

2. **Unit tests** — `test_does_not_grow_unbounded` should continue to pass
   (it doesn't call `marginalize_old_keys`, so the test behaviour is unchanged).
   All other pure-logic tests should still pass.

3. **Graph reset test** — `test_reset_clears_state` verifies that after a reset,
   old keys are not queryable. The `_initialized.discard(k)` change should not
   affect this test.

4. **Pose continuity** — check for any discontinuity events at the
   marginalization boundary. ISAM2 should create smooth linear marginalization
   factors, but this is worth verifying if deep replay becomes available.
