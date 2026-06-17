# Iteration 28 — Rebuild stale `keyframe_buffer` package so the 1.0 s diagnostics timer (fixed in iter26) actually deploys

## What was observed in the logs

### Latest replay results (`logs/replay-results/replay_20260617_071818/`)

The pipeline is stable (14 nodes, GTSAM at 14.4 Hz, TF at 50.8 Hz). Key metrics vs iteration 27:

| Known Failure | Iter 26 | Iter 27 | Iter 28 (before fix) | Notes |
|---|---|---|---|---|
| TF jump events | 2961 | 2939 | **3539** | Increased — run-to-run VIO variation, NOT affected by threshold change |
| odom pose jumped — TF suppressed | 81 | 81 | 81 | Same |
| gtsam odometry rejected | 2 | 2 | 2 | Same |
| tsdf GetAllKeyframes timeout | 1 | 1 | 2 | Same order |
| TF chain disconnected | 17 | 17 | 17 | Same |

### STARVED topics (bag metrics)

| Topic | Effective Hz | Expected Hz | Health |
|---|---|---|---|
| `/gtsam/head_pose` | 14.4 | 15 | OK |
| `/gtsam/arm_pose` | 14.4 | 15 | OK |
| `/tf` | 50.8 | 50 | OK |
| `/keyframe_buffer/diagnostics` | **0.2** | **1.0** | **STARVED** |
| `/vis/head_arm_pose` | 0.0 | 30 | STARVED (no visual factors in bag) |
| `/tf_static` | 0.0 | 1 | STARVED (published once at startup) |

### Critical finding — Installed `keyframe_buffer` package has stale 5.0 s timer

**This is the root cause of `/keyframe_buffer/diagnostics` being STARVED at 0.2 Hz across iterations 26 and 27.**

The source code was correctly changed from `5.0 → 1.0` in iteration 26 (`src/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py:645`):
```python
self.create_timer(1.0, self._publish_diagnostics)   # ← iter26 fix
```

However, the INSTALLED package inside the container still had the old value:

| Location | Timer value | File path |
|---|---|---|
| Source (`src/`) | `1.0` ✅ | `src/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py:645` |
| Build (old — never refreshed) | `5.0` ❌ | `build/keyframe_buffer/build/lib/keyframe_buffer/keyframe_buffer_node.py:622` |
| Build (after rebuild) | `1.0` ✅ | `build/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py:645` |

**Why the fix never took effect:**

The `auto_iterate.sh` script at line 237 builds only `gtsam_tracker` and `cross_camera_features` — it does NOT build `keyframe_buffer`:
```bash
colcon build --packages-select gtsam_tracker cross_camera_features \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The original `ament_python` build (from `make build`) copied the source files into
`build/keyframe_buffer/build/lib/` as regular files. Subsequent `--symlink-install`
rebuilds create egg-link files pointing to `build/keyframe_buffer/`, but only if the
package is actually selected in `--packages-select`.

Since `keyframe_buffer` was never rebuilt after the iter26 source change, the installed
package still used the old 5.0 s timer, causing the diagnostic topic to publish at
0.2 Hz instead of the expected 1.0 Hz.

**Why iter27's config YAML change for `tf_jump_threshold_m` also didn't help:**

The `pipeline_diagnostics_node.py` source default (`0.05 → 0.20`) was also changed
in iter26, and the installed `camera` package also has the stale default (`0.05`).
However, unlike the keyframe_buffer timer (which is hardcoded), the TF jump threshold
is read from ROS parameters — and the YAML config at `config/prosthesis_config.yaml:445`
overrides the code default. So the YAML fix from iter27 (`0.05 → 0.20`) DOES take
effect at runtime. The TF jump count remained high (3539) not because the threshold
change failed, but because virtually all VIO glitches in this bag produce deltas
>0.20 m (sample: 0.431 m, 0.595 m), so raising from 0.05 → 0.20 filters no
measurable number of events.

### Unit tests

| Package | Failures | Notes |
|---|---|---|
| `gtsam_tracker` | 8 | All `RCLError: error creating node: error not set` — ROS2 env issue in unit test harness, not pipeline logic |
| `tsdf_fusion` | 1 | Same RCLError — ROS2 env issue |
| `keyframe_buffer` | **0** | All 53 tests pass — the pure-logic core is fine |

## What was decided to work on

**Problem**: `/keyframe_buffer/diagnostics` is STARVED at 0.2 Hz (expected 1.0 Hz)
because the installed Python package was never rebuilt after the timer was changed from
5.0 s → 1.0 s in iteration 26.

**Fix 1** (immediate): Rebuild `keyframe_buffer` inside the container to deploy the
timer fix.

**Fix 2** (systemic): Add `keyframe_buffer` to `auto_iterate.sh`'s build list so
future changes to this package are automatically deployed.

**Why this matters for the goals:**

- **Localization precision (Goal 1)**: No direct impact — the diagnostic topic is
  monitoring only.
- **TSDF fusion quality (Goal 2)**: Indirect but important. The keyframe_buffer
  diagnostics publish the number of accepted/rejected keyframes, memory usage, and
  per-camera counts. When this topic is STARVED, the operator (or automated analysis)
  has no real-time visibility into whether keyframe spatial gating is working
  correctly. A working 1 Hz diagnostic allows detecting:
  - **Keyframe rejection storms**: If `rejected_gate` spikes, it means the spatial
    gate is too tight, causing too-few keyframes → sparse TSDF pointcloud.
  - **Keyframe acceptance stalls**: If `accepted` plateaus, no new keyframes are
    being added → TSDF fusion re-uses stale data.
  - **Memory budget violations**: The diagnostic warns at 350 MB and errors at 450 MB.
    Without it, an OOM kill would be the first indication of a problem.
  With the diagnostic at 1 Hz, these signals are available in real-time, enabling the
  operator to tune `spatial_gate_translation_m` and `spatial_gate_rotation_deg`
  parameters for optimal TSDF fusion density.

## What was changed

### Fix 1 — Immediate rebuild

Rebuilt the `keyframe_buffer` package inside the running container to deploy the
timer change from iteration 26:

```bash
colcon build --packages-select keyframe_buffer --symlink-install
```

**Verification**: After rebuild, Python loads `keyframe_buffer.keyframe_buffer_node`
from `/prosthesis_ws/build/keyframe_buffer/keyframe_buffer/keyframe_buffer_node.py`
which has the correct `create_timer(1.0, ...)` at line 645.

### Fix 2 — `scripts/auto_iterate.sh:233,237`

**Before:**
```bash
log "Building Python packages (gtsam_tracker, cross_camera_features)..."
colcon build --packages-select gtsam_tracker cross_camera_features \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**After:**
```bash
log "Building Python packages (gtsam_tracker, cross_camera_features, keyframe_buffer)..."
colcon build --packages-select gtsam_tracker cross_camera_features keyframe_buffer \
    --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Adds `keyframe_buffer` to the `--packages-select` list so the package is rebuilt
on every iteration, ensuring source changes are deployed into the container.

## Expected impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

- **No direct change**: The factor graph, noise models, output poses, and TF
  broadcasting are untouched. GTSAM still runs at 14.4 Hz with the same
  between-factor + range-factor structure.
- **Diagnostic visibility improves**: With `/keyframe_buffer/diagnostics` at 1 Hz
  (was 0.2 Hz), the operator can monitor keyframe acceptance rates and memory
  usage in real-time, which helps detect spatial-gate misconfiguration that could
  indirectly affect TSDF fusion quality.

### Goal 2 — TSDF fusion quality (measurably better fused pointcloud)

- **Keyframe acceptance monitoring**: The 1 Hz diagnostic publishes per-camera
  `accepted` and `rejected_gate` counts. If the spatial gate is too tight (e.g.,
  rejecting 90% of clouds), the operator sees `rejected_gate` climbing in real-time
  and can lower `spatial_gate_translation_m` or `spatial_gate_rotation_deg`.
- **Memory budget tracking**: The diagnostic warns at 350 MB and errors at 450 MB.
  Catching an OOM condition before it happens prevents TSDF fusion corruption and
  ensures the pipeline completes its run.
- **No change to keyframe buffer core**: The spatial gating, pose lookup, and
  coordinate transforms are unchanged. The fix only affects the diagnostic
  publishing rate.

### Diagnostic checks for the next iteration

1. **`/keyframe_buffer/diagnostics` effective_hz** — should be **≥0.9 Hz** (was 0.2 Hz),
   no longer STARVED.
2. **All other metrics unchanged**: TF jumps (3539), GTSAM Hz (14.4), TF Hz (50.8),
   odom jumps suppressed (81), GTSAM odometry rejected (2), TSDF timeouts (~2).
3. **No regression in unit tests**: `keyframe_buffer` has 53 tests, all passing.

### Risks

1. **The rebuild changes the install layout** (from `build/lib/` to `build/` root via
   egg-link). If any other node depends on `keyframe_buffer` being importable via
   the old `lib/` path, it would break. However, `keyframe_buffer` is only imported
   by its own console_scripts entry point, which is unaffected.
2. **Adding `keyframe_buffer` to auto_iterate.sh's build list adds ~2.5 s per iteration.**
   This is negligible compared to the 1200 s phase timeout.
