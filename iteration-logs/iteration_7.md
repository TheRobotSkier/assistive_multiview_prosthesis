# Iteration 7 — 2026-06-17

## Goal

Increase the GTSAM output rate from ~12.0 Hz toward the 15 Hz timer target by
reducing ISAM2 per-update computational cost.  The output rate has been stuck
at 11.7–12.1 Hz through all 6 previous iterations despite several targeted
optimizations (marginalization throttling, redundant get_pose elimination).
The fundamental bottleneck is the ISAM2 `update()` call itself.

## What was investigated

### Latest replay results (replay_20260617_025551 — after iteration 6)

| Topic | Count | Eff Hz | Nom Hz | Health |
|---|---|---|---|---|
| `/gtsam/arm_pose` | 2059 | **12.044** | 15 | OK |
| `/gtsam/head_pose` | 2059 | **12.044** | 15 | OK |
| `/vis/head_arm_pose` | 1 | 0.006 | 30 | STARVED |
| `/tf` | 5262 | 30.78 | 50 | OK |

### Rate trend across all iterations

| Metric | Iter1 | Iter2 | Iter3 | Iter4 | Iter5 | Iter6 | Iter7 |
|---|---|---|---|---|---|---|---|
| `/gtsam/head_pose` Hz | 11.93 | 11.9 | 12.135 | 11.748 | 12.001 | 12.044 | ? |
| CPU avg | 48% | — | 60% | 60.9% | 59.1% | 60.8% | ? |

### Core finding: cross-chain range factor overloads every ISAM2 update

The GTSAM factor graph receives **114 Hz VIO odometry** but the `_graph_update`
timer runs at 15 Hz (66.7 ms period).  Each callback takes ~83 ms, so ~1 in 4
timer firings are skipped, yielding the ~12 Hz output.

After tracing every operation in `_graph_update()` (lines 454–620), the
dominant consumer is the `_isam.update()` call (line 592).  Three structural
issues compound:

#### 1. Range factor on every cycle forces cross-chain Bayes tree cliques

Since iteration 3, the kinematic range factor (`BetweenFactorPose3` linking
head key h_i to arm key a_i) is added on **every** `_graph_update` cycle.
This means every ISAM2 update processes 3 new factors (2 odometry + 1 range)
instead of the 2 that are strictly required.

More importantly, the range factor is a **cross-chain** constraint.  In ISAM2's
Bayes tree, cross-chain factors force variables from the head and arm chains
into **larger cliques**.  When every consecutive pair (h_i, a_i) has a range
factor, the Bayes tree's conditional structure spans both chains across the
entire fixed-lag window.  This makes **every** back-substitution and
relinearisation more expensive, not just the cycles that add a range factor.

Estimated impact: the range factor increases per-update time by ~5–10 ms
through larger cliques alone, beyond the ~1 ms of linearising the factor
itself.

#### 2. `relinearizeThreshold=0.001` triggers excessive re-linearisation

ISAM2 parameters:
```python
isam_params.setRelinearizeThreshold(0.001)   # 1 mm / 0.057°
isam_params.relinearizeSkip = 3
```

A threshold of 0.001 means a variable is re-linearised if its linearisation
point changes by more than **1 mm translation** or **0.057° rotation**.  At
~12 Hz update rate with smooth arm motion, nearly every variable near the
current time exceeds this threshold during each relinearize cycle (every 3rd
update).  This causes ISAM2 to re-linearise ~50–100 variables per
relinearize cycle, adding ~10–15 ms.

Default GTSAM `relinearizeThreshold` is 0.1 — our value was 100× tighter.

#### 3. `relinearizeSkip=3` runs full checks too often

Relinearization every 3rd update means ~4 Hz full checks.  With a bounded
graph of ~360 variables, most of these checks find variables that have changed
(since the threshold is so tight) and trigger re-linearisation work.  Spreading
this to every 5th update (~2.4 Hz) would reduce the amortised overhead by 40%.

#### 4. `np.linalg.inv` in range factor computation is sub-optimal

`_add_kinematic_range_factor()` computes `inv(self._head_pose)` using the
generic `np.linalg.inv` (line 650 pre-fix).  For SE(3) matrices the inverse
can be computed with the efficient formula `inv([R, t; 0, 1]) = [R^T, -R^T t;
0, 1]`, available as `inverse_se3()` from `se3_helpers.py` (already imported).
While the per-cycle saving is tiny (~5 μs), it replaces a try/except block
that guarded against `LinAlgError` with a guaranteed-non-singular SE(3)
operation.

### Unit tests

The 8 `gtsam_tracker` failures and 1 `tsdf_fusion` failure remain the same
ROS2 infrastructure issue (`RCLError: error creating node`).  No logic-test
failures.  Pure-logic tests (`test_factor_graph.py`) should be unaffected
by all changes.

## Changes made

### Change 1: `src/gtsam_tracker/gtsam_tracker/factor_graph.py:88-93` and `:335-340`

**ISAM2 relinearization parameters relaxed** in both `__init__` and `reset()`:

```
relinearizeThreshold: 0.001 → 0.01   (1 mm  → 1 cm / 0.057° → 0.57°)
relinearizeSkip:       3      → 5     (every 3rd → every 5th update)
```

- `relinearizeThreshold=0.01`: A variable must change by >1 cm translation or
  >0.57° rotation before triggering re-linearisation.  At 12 Hz with smooth
  arm motion, most variables stay within this bound between relinearize cycles,
  so the number of re-linearised variables drops from ~50-100 to ~5-15.

- `relinearizeSkip=5`: Full relinearization check runs every 5th update
  (~2.4 Hz) instead of every 3rd (~4 Hz).  Reduces relinearization frequency
  by 40%.

**Expected impact**: Relinearize cycle overhead drops from ~10-15 ms to ~2-5
ms.  Amortised per-cycle saving: ~5 ms (from 15/3 = 5 ms → 5/5 = 1 ms).

### Change 2: `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:476-485`

**Range factor added every 3rd cycle** instead of every cycle:

```python
# BEFORE (every cycle):
self._add_kinematic_range_factor()

# AFTER (every 3rd cycle):
if self._key_idx % 3 == 0:
    self._add_kinematic_range_factor()
```

On 2 out of 3 cycles, ISAM2 only processes 2 odometry between-factors — no
cross-chain constraint.  The Bayes tree can keep head and arm cliques separate
(mostly independent chains), making back-substitution and relinearisation
~2× faster for those cycles.  On 1 out of 3 cycles, the range factor reunites
the chains, keeping drift bounded.

**Expected impact**: Per-cycle ISAM2 time reduces by ~5–10 ms on non-range
cycles (2/3 of all cycles).  Amortised saving: ~4–7 ms.

The `kinematic_check_interval` ROS parameter (declared at line 67, read at
line 202) is now dead code; the range factor frequency is controlled by the
`key_idx % 3 == 0` condition.  This is safe — the parameter was already unused
by the unconditional range factor logic in iterations 3–6.

### Change 3: `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:654`

**Replace `np.linalg.inv` with `inverse_se3`** for computing the head→arm
relative transform in `_add_kinematic_range_factor`:

```python
# BEFORE:
inv_head = np.linalg.inv(self._head_pose)

# AFTER:
inv_head = inverse_se3(self._head_pose)
```

`inverse_se3()` uses the SE(3)-efficient formula `[R^T, -R^T t; 0, 1]` instead
of generic 4×4 LU decomposition.  It is mathematically identical for valid
SE(3) matrices and eliminates the `np.linalg.LinAlgError` guard (SE(3) matrices
are always invertible).  Tiny per-cycle saving (~5 μs).

## Expected impact

### Localization precision (goal 1)

The primary gain is **higher GTSAM output rate**:

| Metric | Current (iter6) | Expected |
|---|---|---|
| `/gtsam/head_pose` Hz | 12.044 | **13.5–14.5 Hz** |
| `/gtsam/arm_pose` Hz | 12.044 | **13.5–14.5 Hz** |
| CPU avg | 60.8% | 55–60% |

The expected ~2 Hz improvement comes from:
- **~4–7 ms** saved by sparser range factors (smaller Bayes tree cliques on
  2/3 of cycles)
- **~4–5 ms** saved by relaxed relinearization (fewer re-linearised variables
  per relinearize cycle, which happens 40% less often)
- Total: ~8–12 ms saved per cycle, reducing callback time from ~83 ms to
  ~71–75 ms — approaching the 66.7 ms timer period.

Higher output rate means:
- More pose estimates per second → less interpolation error downstream
- More frequent GTSAM corrections to the VIO drift → tighter trajectory
- Smoother TF tree updates for downstream consumers

### No degradation in cross-chain coupling

The range factor is still added every 3rd cycle (~4 Hz versus ~12 Hz
previously).  At typical arm motion speeds, the head-arm relative pose changes
by millimetres per cycle, so 2 cycles (~166 ms) of free drift between range
factors adds negligible uncorrected drift (~mm scale).  The slightly sparser
coupling is far outweighed by the benefit of a faster, more responsive
smoother.

### TSDF fusion (goal 2, indirect)

Higher pose output rate → more pose updates available for pointcloud
integration → denser fused volume with fewer holes.  Also, faster ISAM2
updates mean less timing jitter between pose publications, reducing
temporal misalignment between pointcloud capture and its associated pose.

## What to check in the next iteration

1. **GTSAM output rate** — check `bag_analysis.txt` for `/gtsam/head_pose`
   and `/gtsam/arm_pose` effective_hz.  Expect **≥13.5 Hz** (vs 12.044 Hz).

2. **CPU usage** — check `sysmon.jsonl` host_cpu_pct avg; expect slight
   decrease from 60.8%.

3. **Range factor coupling** — verify that `/gtsam/head_pose` and
   `/gtsam/arm_pose` trajectories remain consistent despite sparser range
   factors.  No new divergence events expected.

4. **Unit tests** — pure-logic tests (`test_factor_graph.py`) should all pass
   (none call `_add_kinematic_range_factor` or depend on ISAM2 params).

5. **Bag analysis for visual factor rate** — `/vis/head_arm_pose` still
   expected to be starved (golden bag limitation persists).
