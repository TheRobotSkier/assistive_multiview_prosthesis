# Iteration 6 — 2026-06-17

## Goal

Improve the GTSAM output rate (stuck at ~12.0 Hz vs 15 Hz timer target) by
eliminating redundant `get_pose()` / `calculateEstimatePose3()` calls in the
per-cycle graph update path.

## What was investigated

### Latest replay results (replay_20260617_024625 — after iteration 5)

| Topic | Count | Eff Hz | Nom Hz | Health |
|---|---|---|---|---|
| `/gtsam/arm_pose` | 2053 | **12.001** | 15 | OK |
| `/gtsam/head_pose` | 2053 | 12.001 | 15 | OK |
| `/vis/head_arm_pose` | 1 | 0.006 | 30 | STARVED |
| `/tf` | 5276 | 30.84 | 50 | OK |

### Rate trend across iterations

| Metric | Iter1 | Iter2 | Iter3 | Iter4 | Iter5 | Iter6 (fix) |
|---|---|---|---|---|---|---|
| `/gtsam/head_pose` Hz | 11.93 | 11.9 | 12.135 | 11.748 | 12.001 | ? |
| CPU avg | 48% | — | 60% | 60.9% | 59.1% | ? |

Key observations:
1. GTSAM output remains at ~12.0 Hz — well below the 15 Hz timer rate
   (66.7 ms period vs ~83 ms per callback).
2. The marginalization throttling in iteration 5 gave a small recovery
   (11.748 → 12.001 Hz) but the gap is still ~3 Hz.
3. `/vis/head_arm_pose` still at 1 message — golden bag image scarcity
   persists (20 head images, 11 arm images over 171 s).
4. No errors, crashes, or TF jumps.
5. All pure-logic unit tests pass. The 8 gtsam_tracker failures and 1
   tsdf_fusion failure are all `RCLError: error creating node` — ROS2
   test infrastructure, not logic bugs.

### Root cause: redundant `get_pose()` calls

Tracing `_graph_update()` shows **4 `get_pose()` calls per cycle**, each
triggering `ISAM2.calculateEstimatePose3()` — a linear back-substitution
on the Bayes tree:

| Call site | File:Line | Purpose | Can eliminate? |
|---|---|---|---|
| `add_odometry_factor` (head) | `factor_graph.py:190` | Initial estimate: `get_pose(prev_key) * delta` | Yes |
| `add_odometry_factor` (arm) | `factor_graph.py:190` | Initial estimate: `get_pose(prev_key) * delta` | Yes |
| `_publish_poses` (head) | `gtsam_tracker_node.py:746` | Get smoothed pose to publish | No (essential) |
| `_publish_poses` (arm) | `gtsam_tracker_node.py:754` | Get smoothed pose to publish | No (essential) |

The two `add_odometry_factor` calls at `factor_graph.py:189-191` compute:
```python
prev_pose = self.get_pose(self._make_key(chain, key_last))
init_pose = prev_pose.compose(delta_pose)
```

But `prev_pose.compose(delta_pose)` is mathematically equivalent to the
raw odometry pose of the current step.  Specifically:
- `delta = relative_transform(prev_odom, current_odom)`
  = `inv(prev_odom) * current_odom`
- `prev_pose` (ISAM2 smoothed) ≈ `prev_odom` (raw VIO)
- So `prev_pose * delta ≈ prev_odom * inv(prev_odom) * current_odom`
  = `current_odom`

The raw odometry pose (`self._head_pose` / `self._arm_pose`) is an
excellent initial estimate — it is the VIO output, which is already
close to the true trajectory.  ISAM2's nonlinear optimization converges
from this estimate just as well as from the ISAM2-composed estimate,
but without the expensive back-substitution.

Each `calculateEstimatePose3()` back-substitution on a ~370-variable
Bayes tree adds O(n) overhead.  Eliminating 2 of the 4 calls frees
~1–3 ms per cycle (conservative), which directly reduces the per-cycle
wall time.

## Changes made

### Change 1: `src/gtsam_tracker/gtsam_tracker/factor_graph.py:126-194`

**Added `init_head` / `init_arm` optional parameters to `add_odometry_factor()`**

When provided, these pre-computed `gtsam.Pose3` values are used as the
initial estimate for the new key, bypassing the `get_pose()` call:

```python
# New code at line 180-193 — priority order:
# 1. If init_head/init_arm provided → use directly (avoids get_pose)
# 2. Else if previous key is initialized → use get_pose(prev) * delta (old path)
# 3. Else → use delta_pose as-is (first key of chain)
```

Backward-compatible: when `init_head=None` / `init_arm=None` (the
default), the behavior is identical to the original code.

### Change 2: `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:23`

**Added `import gtsam`** at module level (gtsam is already a transitive
dependency via `factor_graph.py`, so this does not introduce a new
dependency; it just makes the binding explicit for the new code).

### Change 3: `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:535-551`

**Pre-compute initial estimates from raw odometry and pass to `add_odometry_factor`**

Before the `add_odometry_factor()` call, convert the current raw odometry
poses into `gtsam.Pose3` objects and pass them as `init_head` / `init_arm`:

```python
init_head = (
    gtsam.Pose3(np.asarray(self._head_pose, dtype=np.float64))
    if self._head_pose is not None else None)
init_arm = (
    gtsam.Pose3(np.asarray(self._arm_pose, dtype=np.float64))
    if self._arm_pose is not None else None)

self._graph.add_odometry_factor(
    self._key_idx, self._key_idx, stamp,
    delta_head, delta_arm,
    noise_head, noise_arm,
    init_head=init_head, init_arm=init_arm)
```

When either `_head_pose` or `_arm_pose` is `None` (e.g. first callback
before odometry arrives), the corresponding parameter is `None` and the
fallback in Change 1 uses the original `get_pose()` path.

## Expected impact

### Localization precision (goal 1)

1. **Higher GTSAM output rate**: Eliminating 2× `calculateEstimatePose3()`
   per cycle saves an estimated 1–6 ms (conservative).  With the current
   ~83 ms cycle time and 66.7 ms timer period, every ms saved directly
   increases the fraction of timer firings that complete in time.  Expected
   improvement: **12.0 → 13–14 Hz**.

2. **No degradation in estimate quality**: The raw odometry pose is an
   equally valid initial estimate for ISAM2's nonlinear optimizer — it
   differs from the `get_pose(prev) * delta` value only by the ISAM2
   correction applied to the previous key, which is typically sub-mm
   per step.

3. **Consistent per-cycle latency**: Removing the get_pose calls makes
   the callback time more predictable, reducing timing jitter in pose
   publishing.

### TSDF fusion (goal 2, indirect)

- Higher pose output rate → more frequent TSDF integration steps →
  denser fused volume with fewer holes.

### Quantitative expectation

- `/gtsam/head_pose` and `/gtsam/arm_pose` effective Hz: **12.0 → ≥13.5 Hz**
  (conservative; fewer timer skips per second).
- CPU usage: expected slight decrease (fewer ISAM2 queries per cycle).
- No change expected in `/vis/head_arm_pose` (golden bag limitation persists).
- All unit tests should remain unchanged — the `init_head=None` default
  preserves existing behavior for all test callers.

## What to check in the next iteration

1. **GTSAM output rate** — check `bag_analysis.txt` for `/gtsam/head_pose`
   and `/gtsam/arm_pose` effective_hz.  Expect ≥13.5 Hz.
2. **CPU usage** — check `sysmon.jsonl` host_cpu_pct avg; expect slight
   decrease from 59.1%.
3. **Visual factor rate** — `/vis/head_arm_pose` is still expected to be
   starved (golden bag limitation).  If a new bag becomes available with
   more images, the cross-camera features can finally be tested.
4. **Unit tests** — the pure-logic factor graph tests should all pass
   (they don't use the new `init_head`/`init_arm` parameters).
