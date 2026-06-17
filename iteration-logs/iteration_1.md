# Iteration 1 — 2026-06-17

## Goal

Improve GTSAM localization precision by fixing a noise-model ordering bug in the
kinematic range factor, which was preventing the head–arm distance constraint
from activating effectively.

## What was investigated

### Logs & replay results

No prior replay logs exist (first iteration). The unit tests in
`logs/test-results/` were absent.  The codebase was explored instead.

### Pipeline architecture established

- `gtsam_tracker/factor_graph.py` — Two-chain (head+arm) ISAM2 fixed-lag smoother.
- `gtsam_tracker/gtsam_tracker_node.py` — ROS node that feeds odometry & visual
  factors into the graph.
- `keyframe_buffer/` — Maintains a sliding buffer of keyframes from each camera.
- `pointcloud_fusion/` — Publishes merged pointclouds.
- `tsdf_fusion/` — GPU-accelerated TSDF integration (Open3D / bespoke kernels).
- `cross_camera_features/` — SIFT matching between camera pairs for loop
  closure / cross-chain visual constraints.

### Bug found: swapped translation/rotation sigmas in `add_range_factor`

Location: `src/gtsam_tracker/gtsam_tracker/factor_graph.py:281-284` (pre-fix).

The `add_range_factor` method adds a `BetweenFactorPose3` between head and arm
at the same timestamp.  Its noise model was:

```python
np.array([10.0, 10.0, 10.0,     # ← translation sigma = 10 m (≈no constraint)
          max_distance,          # ← rotation sigma  = 1 rad (moderate)
          max_distance,
          max_distance])
```

GTSAM's `BetweenFactorPose3` error vector ordering is **`[tx, ty, tz, rx, ry, rz]`**
— translation first, rotation second.  The intended semantics (documented in the
same method's docstring and comment) are that the range factor should constrain
**translation** to keep the two kinematic chains from drifting apart, while
leaving **rotation** essentially free.

The swapped values meant:
- Translation sigma was `10.0 m` — far too loose to prevent drift between the
  head and arm chains.
- Rotation sigma was `max_distance` (e.g. `1.0 m` — absurd; `Sigmas` treats all
  six values as the same unit, but radians for rotation components) — though in
  practice this was mostly harmless because `BetweenFactorPose3` errors for
  rotation are in radians, so `1.0 rad` is also fairly weak.

The net effect: the kinematic "rubber band" that should keep the head and arm
trajectories coupled was **effectively disabled**, allowing each chain to drift
independently in translation.  This directly harms localization precision
because the two-camera system loses the geometric consistency constraint.

## Changes made

**File:** `src/gtsam_tracker/gtsam_tracker/factor_graph.py:281-288`

Swapped the noise-model array values so that translation sigma =
`max_distance` and rotation sigma = `10.0 rad`, matching the documented
intent and GTSAM's error-vector ordering convention:

```python
# OLD (buggy):
np.array([10.0, 10.0, 10.0,
          float(max_distance), float(max_distance), float(max_distance)])

# NEW (fixed):
np.array([float(max_distance), float(max_distance), float(max_distance),
          10.0, 10.0, 10.0])
```

Also added an explicit comment documenting the GTSAM `BetweenFactorPose3`
error ordering `[tx, ty, tz, rx, ry, rz]` as a reference for future
maintainers.

## Expected impact

**Localization precision:**
- The kinematic range factor will now actively constrain head–arm translation
  drift, keeping the two pose chains coupled within roughly `max_distance`
  metres.
- GTSAM smoothing should produce lower trajectory RMS error and less drift
  between the two camera chains.
- In replay metrics: better `odom→gtsam` agreement (lower ATE/RPE), fewer
  divergence events between head and arm estimates.

**TSDF fusion (indirect):**
- With tighter GTSAM estimates, the poses fed to the TSDF integrator are more
  consistent.  This should produce a denser, more spatially coherent fused
  pointcloud with fewer outliers.

## What to check in the next iteration

1. Run the test suite — `test_range_pulls_together` should still pass (the fix
   makes the factor actually pull, which is strictly stronger).
2. Examine replay `bag_analysis.txt` for `effective_hz` of `/gtsam_pose_*` topics
   — no change expected from this fix alone.
3. Examine replay `log_metrics.json` for trajectory RMS / ATE — expect
   improvement.
4. Check for topic starvation (low `effective_hz`) — that would be a separate
   issue to investigate next.
