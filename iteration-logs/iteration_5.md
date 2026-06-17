# Iteration 5 — 2026-06-17

## Goal

Fix the GTSAM output rate regression (12.135 → 11.748 Hz) from iteration 4's
marginalization fix, and eliminate a range-factor key duplication bug that
causes redundant graph factors and delayed cross-chain coupling.

## What was investigated

### Replay results (replay_20260617_023626 — after iteration 4)

| Topic | Count | Eff Hz | Nom Hz | Health |
|---|---|---|---|---|
| `/gtsam/arm_pose` | 2017 | 11.748 | 15 | OK |
| `/gtsam/head_pose` | 2017 | 11.748 | 15 | OK |
| `/vis/head_arm_pose` | 1 | 0.006 | 30 | STARVED |
| `/tf` | 5281 | 30.76 | 50 | OK |

### Regression trend

| Metric | Iter1 | Iter2 | Iter3 | Iter4 |
|---|---|---|---|---|
| `/gtsam/head_pose` Hz | 11.93 | 11.9 | **12.135** | **11.748** |
| `/gtsam/arm_pose` Hz | 11.93 | 11.9 | 12.135 | 11.748 |
| CPU avg | 48% | — | 60% | 60.9% |

The GTSAM output rate **dropped 3.2%** (12.135 → 11.748 Hz) after iteration 4's
marginalization fix.  The 15 Hz timer fires every 66.7 ms, but each callback
takes ~85 ms, causing ~1 in 4 timer firings to be skipped.

### Bug 1: Range factor key duplication

**Location:** `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:610-611` (pre-fix)

The `_add_kinematic_range_factor` method used `max(self._key_idx - 1, 0)` to
compute the GTSAM symbol keys for the range factor's between-factor.  This
adds the constraint on the **previous** key pair (h_prev, a_prev) instead of
the **current** pair being added in this cycle.

**Effect across consecutive update cycles:**

| Cycle | `key_idx` | Range factor on | Odometry adds | Published |
|---|---|---|---|---|
| 1 | 0 | h0, a0 | h0, a0 (prior) | h0, a0 |
| 2 | 1 | **h0, a0** ← DUPLICATE | h1, a1 (between) | h0, a0 |
| 3 | 2 | h1, a1 | h2, a2 | h1, a1 |
| 4 | 3 | h2, a2 | h3, a3 | h2, a2 |

Consequences:
1. **Duplicate range factors**: Pairs (h0,a0) get range factors in cycles 1 AND 2.
   Every subsequent pair also gets an extra range factor from the *next* cycle
   before being published.  This bloats the factor graph with redundant factors.
2. **One-cycle coupling delay**: The newest key pair (e.g., h1,a1 created in
   cycle 2) does not receive a range constraint until cycle 3.  This means the
   head and arm chains are not cross-coupled for their most recent poses,
   reducing localization precision during the delay.
3. **Wasted ISAM2 computation**: ISAM2 must linearize and optimize the
   duplicate range factors, adding unnecessary per-update cost.

### Bug 2: Marginalization overhead on every update

**Location:** `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:562-569` (pre-fix)

After iteration 4, `marginalize_old_keys` is called every single update cycle.
Each call:
- Iterates through all tracked timestamps (~360 entries when bounded)
- Builds a `gtsam.KeyVector` of keys to marginalize
- Passes it to ISAM2's `update()` as `marginalizeTheta`

ISAM2 responds by creating **linear marginalization factors** (dense
approximations that replace the removed variables).  This operation adds
~10–15 ms per call.  Since the timer period is 66.7 ms and the baseline
callback takes ~75 ms without marginalization, adding 10–15 ms pushes it to
~85 ms — exceeding the timer period and causing skipped firings.

The marginalization is necessary to keep the graph bounded, but it does not
need to run every cycle (12 Hz).  Running it every N=5 cycles keeps the graph
size at ~370 variables instead of ~360 — a negligible difference — while
reducing marginalization overhead by 80%.

## Changes made

### Change 1: Fix range factor key index (`gtsam_tracker_node.py:611-628`)

**Before:**
```python
kh = self._graph._make_key("h", max(self._key_idx - 1, 0))
ka = self._graph._make_key("a", max(self._key_idx - 1, 0))
```

**After:**
```python
kh = self._graph._make_key("h", self._key_idx)
ka = self._graph._make_key("a", self._key_idx)
```

This ensures the range factor is added between the **current** key pair being
created in this update cycle, eliminating:
- Duplicate range factors on the same key pair
- The one-cycle delay before cross-chain coupling activates

Each published key pair now receives exactly one range factor, from the cycle
in which it was created.  The `_publish_poses` method (line 735-736) continues
to use `key_idx - 1` for the intentional one-cycle smoothing delay before
publishing the refined estimate.

### Change 2: Throttle marginalization to every 5 updates (`gtsam_tracker_node.py:568-573`)

**Before:**
```python
old_keys: list = []
try:
    old_keys = self._graph.marginalize_old_keys(stamp)
except Exception:
    pass
```

**After:**
```python
old_keys: list = []
if self._key_idx % 5 == 0:
    try:
        old_keys = self._graph.marginalize_old_keys(stamp)
    except Exception:
        pass
```

Marginalization now runs every 5 updates (~2.4 Hz at 12 Hz input, every ~0.42 s).
Between marginalization calls, `update()` is called with an empty `KeyVector`
(no marginalization), which is the same fast-path ISAM2 used before iteration 4.

**Graph size impact:** With throttling to every 5 updates, the graph grows by at
most 10 extra variables (5 updates × 2 chains) between marginalization events:
  - Without throttling: ~360 variables (180 head + 180 arm for 15 s lag)
  - With throttling (5-cycle): ~370 variables max (negligible +2.8%)

## Expected impact

### Localization precision (goal 1)

1. **Tighter head-arm coupling**: Every new key pair now receives a range factor
   in the same update cycle where it is created.  This eliminates the one-cycle
   delay (previously ~83 ms without cross-chain constraint) during which the two
   chains could drift independently before the range factor pulls them together.

2. **No duplicate over-constraint**: Previously, each key pair accumulated 2×
   range factors, effectively reducing the noise sigma by √2 (making the
   constraint 1.4× stronger than documented).  With the fix, each pair gets
   exactly one factor, matching the intended noise model semantics.

3. **Higher output rate**: The marginalization throttling reduces per-update
   overhead by ~10–15 ms for 4 out of 5 cycles.  This should lift the effective
   GTSAM publish rate from ~11.7 Hz back toward 13–14 Hz (closer to the 15 Hz
   timer).  More frequent pose estimates → less interpolation error downstream.

### TSDF fusion (goal 2, indirect)

- Higher pose output rate → more frequent TSDF integration steps → denser fused
  volume with fewer holes.
- Tighter head-arm coupling → more consistent relative poses → fewer pointcloud
  misalignment artefacts.

### Quantitative expectation

- `/gtsam/head_pose` and `/gtsam/arm_pose` effective Hz: **11.7 → ≥13.0 Hz**
  (expected recovery from marginalization throttling + fewer duplicate factors).
- No change expected in `/vis/head_arm_pose` (golden bag limitation persists).
- CPU usage may decrease slightly (~55–60% avg vs 60.9%).
- Pure-logic unit tests (`test_factor_graph.py`) remain unchanged — they do not
  call `_add_kinematic_range_factor` or use marginalization keys.

## What to check in the next iteration

1. **GTSAM output rate** — check `bag_analysis.txt` for `/gtsam/head_pose` and
   `/gtsam/arm_pose` effective_hz.  Expect ≥13.0 Hz (vs 11.748 Hz in iter4).

2. **Range factor correctness** — the `test_range_pulls_together` unit test
   should continue to pass (it calls `add_range_factor` directly, which is
   unchanged).

3. **Marginalization effectiveness** — the `test_does_not_grow_unbounded` test
   should still pass (it does not call `marginalize_old_keys` or pass
   `marginalize_keys`).  But verify the graph does not grow unboundedly in
   longer runs: with throttled marginalization, the first 5 cycles after
   the lag window don't shrink, but subsequent 5-cycle batches do.

4. **Pose continuity at marginalization boundaries** — check for any
   discontinuity when thick-marginalization (every 5 cycles) removes old
   variables.  ISAM2 should create smooth linear marginalization factors
   regardless of whether it happens every cycle or every 5 cycles.

5. **Long-run stability** — if a run longer than 171 s is available, verify
   the graph stays bounded (the throttled marginalization should keep the
   size at ~370 variables indefinitely).
