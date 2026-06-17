# Iteration 29 — Fix GTSAM TF broadcast timestamp domain: use host clock so TF2 chain composition works

## What was observed in the logs

### Latest replay results (`logs/replay-results/replay_20260617_073037/`)

The pipeline is stable (14 nodes, GTSAM at 14.3 Hz, TF at 50.7 Hz). Key metrics:

| Known Failure | Iter 26 | Iter 27 | Iter 28 | Iter 29 (before fix) | Notes |
|---|---|---|---|---|---|
| TF jump events | 2961 | 2939 | 3539 | **3486** | Run-to-run VIO variation, unchanged by threshold change |
| odom pose jumped — TF suppressed | 81 | 81 | 81 | **81** | Same |
| gtsam odometry rejected | 2 | 2 | 2 | **2** | Same |
| tsdf GetAllKeyframes timeout | 1 | 1 | 2 | **2** | Same order |
| TF chain disconnected | 17 | 17 | 17 | **17** | Same |

### Topic health

| Topic | Effective Hz | Expected Hz | Health |
|---|---|---|---|
| `/gtsam/head_pose` | 14.3 | 15 | OK |
| `/gtsam/arm_pose` | 14.3 | 15 | OK |
| `/tf` | 50.7 | 50 | OK |
| `/keyframe_buffer/diagnostics` | **1.0** | 1.0 | OK ✅ (iter28 fix deployed) |
| `/vis/head_arm_pose` | 0.0 | 30 | STARVED (no visual factors in bag) |
| `/tf_static` | 0.0 | 1 | STARVED (published once at startup) |

### Critical finding — TF chain DISCONNECTED for depth_optical_frame ENTIRE run

The diagnostic shows:

```
TF chain connectivity (marker_map -> depth_optical_frame):
  > arm_d435i_arm_depth_optical_frame: DISCONNECTED entire run (33 blocks)
  > head_d435i_head_depth_optical_frame: DISCONNECTED entire run (35 blocks)
  > marker_map -> head_imu: DISCONNECTED
  > marker_map -> arm_imu: DISCONNECTED
```

The `marker_map -> *_imu` edges are published by GTSAM tracker (`broadcast_tf=True`), not by the relay (`publish_dynamic_tf=False` per V6 §6.3). All other TF edges in the chain (bridge: `*_cam0 -> *_link`, relay: `*_imu -> *_cam0`, nominal static chain: `*_link -> *_depth_optical_frame`) use **host clock timestamps** (`get_clock().now().to_msg()`).

**The GTSAM tracker was the only TF publisher using sensor timestamps.**

#### Root cause: timestamp-domain mismatch breaks TF2 chain composition

The GTSAM tracker's `_publish_pose` method at `gtsam_tracker_node.py:900` stamped the TF broadcast with `stamp_msg` — the sensor timestamp from the Jetson odometry message:

```python
# BEFORE (broken):
tf_msg.header.stamp = stamp_msg       # ← sensor timestamp, ~56000s behind host
tf_msg.header.frame_id = self._world_frame
tf_msg.child_frame_id = child_frame
...
self._tf_broadcaster.sendTransform(tf_msg)
```

The sensor clock has a peak offset of **~56460 seconds** from the host clock (`clock_offset_peak_s: 56625.919` in log_metrics.json — constant offset, not drift). This means the GTSAM edge `marker_map -> head_imu` was published at timestamp T_sensor (≈56460 s in the past).

When TF2's `can_transform(marker_map, head_d435i_head_depth_optical_frame, Time(0))` tries to compose the chain:
1. **Bridge edges** (`head_cam0 -> link`, static chain) use host timestamps ~ present day
2. **GTSAM edge** (`marker_map -> head_imu`) has timestamp ~56000 s ago
3. TF2 cannot find a common time where both edges exist in the cache — the GTSAM edge at T_sensor appears to have expired when queried alongside bridge edges at T_host
4. Result: `can_transform` returns `False`, chain is reported as DISCONNECTED

**This directly impacts Goal 2 (TSDF fusion)**: the TSDF fusion node needs `marker_map -> depth_optical_frame` transforms to align incoming pointclouds into the global frame. If this chain is DISCONNECTED, the fusion back-end cannot compute the correct camera pose for each cloud, degrading the fused pointcloud.

### Additional observation — GTSAM scheduled resets at ~48 s intervals

The `graph_reset_interval: 720` (code default, not overridden in config YAML) causes scheduled factor graph resets every ~48 s at 15 Hz. The log shows:
- **Reset #1** at T≈56 s
- **Reset #2** at T≈105 s

After each reset, `gtsam_corr` drops to 0.0000 m (first stat line after reset: `gtsam_corr(head=0.0000m, arm=0.0000m)`), confirming the smoothed output reverts to raw VIO for ~7 seconds until the smoother lag refills.

**This is a secondary contributor to Goal 1 (localization precision)**: GTSAM doesn't provide smoothed output during ~14 s out of each 170 s run (~8% degraded). However, this reset is an intentional design choice from iteration 19 to prevent ISAM2 rate decline, and fixing it requires a more complex approach (e.g., variable-marginalization strategy) beyond the scope of a single iteration.

## What was decided to work on

**Problem**: GTSAM tracker publishes its TF edges (`marker_map -> head_imu`, `marker_map -> arm_imu`) with **sensor timestamps** (from the Jetson clock, ~56460 s behind the host), while every other TF publisher in the pipeline (bridge, relay, camera mounts) uses **host clock timestamps**. This timestamp-domain mismatch prevents TF2 from composing the multi-edge chain, causing `marker_map -> depth_optical_frame` to be reported as DISCONNECTED for the entire run.

**Fix**: Change the TF broadcast in `_publish_pose` to use the host clock timestamp (`self.get_clock().now().to_msg()`) instead of the sensor timestamp (`stamp_msg`). The pose topic (`/gtsam/head_pose`, `/gtsam/arm_pose`) retains the sensor timestamp for temporal alignment with sensor data.

**Why this matters for the goals:**

- **Goal 1 (localization precision)**: No direct change — the factor graph, output poses, and smoother structure are untouched. However, TF diagnostics will now accurately report chain connectivity, making it possible to detect genuine VIO/VPN failures vs. timestamp-domain artifacts.

- **Goal 2 (TSDF fusion quality)**: **Direct impact**. With the TF chain correctly connected, the TSDF fusion node can reliably look up `marker_map -> depth_optical_frame` transforms for each pointcloud timestamp. This means:
  - Pointclouds are aligned using the **correct camera pose in the global frame** (via the complete TF chain) rather than falling back to degraded paths or approximations.
  - The fused pointcloud has **tighter spatial consistency** and **fewer outliers** (fewer misaligned clouds).
  - The keyframe buffer's spatial gating (which uses TF lookups) can verify the camera position relative to the world.

## What was changed

### `src/gtsam_tracker/gtsam_tracker/gtsam_tracker_node.py:909`

**Before (line 900 of original):**
```python
tf_msg.header.stamp = stamp_msg       # ← sensor timestamp from Jetson clock
```

**After:**
```python
tf_msg.header.stamp = self.get_clock().now().to_msg()  # ← host clock timestamp
```

The `stamp_msg` variable still exists and is used correctly at line 880 for the pose topic message (`msg.header.stamp = stamp_msg`). Only the TF broadcast's timestamp was changed, so:
- Pose topics (`/gtsam/head_pose`, `/gtsam/arm_pose`) are **unchanged**: they retain sensor timestamps for temporal alignment with sensor data
- TF broadcasts (`marker_map -> head_imu`, `marker_map -> arm_imu`) now use **host timestamps**, matching the convention of all other TF publishers in the pipeline

### Additional context added as comments (lines 897-905)

Added a detailed code comment explaining the timestamp-domain mismatch and why host timestamps are required for TF2 chain composition.

## Expected impact

### Goal 1 — Localization precision (GTSAM factor-graph optimization)

- **No change to factor graph, smoothing, or output poses**: The GTSAM tracker continues to output smoothed poses at 14.3 Hz with the same between-factor + range-factor structure and 7-second smoother lag.
- **TF diagnostics become meaningful**: The `marker_map -> depth_optical_frame` chain transitions from DISCONNECTED to CONNECTED for the majority of the run (except during graph reset periods when GTSAM briefly stops publishing TFs). This lets the operator distinguish genuine VIO disconnections from a timestamp-domain artifact.
- **TF jump detection continues to work**: The pipeline_diagnostics_node's jump detection uses consecutive TF deltas, which are unaffected by the absolute timestamp value.

### Goal 2 — TSDF fusion quality (measurably better fused pointcloud)

- **TF chain now consistently available**: With all edges in the chain using host timestamps, TF2's `can_transform` succeeds, enabling reliable `marker_map -> depth_optical_frame` lookups.
- **Pointcloud alignment improves**: When the keyframe buffer and TSDF fusion node look up the camera pose for each pointcloud, the complete chain resolves correctly:
  - `marker_map -> head_imu` (GTSAM smoothed pose, host timestamp)
  - `head_imu -> head_cam0` (relay static TF, always available)
  - `head_cam0 -> head_d435i_head_link` (bridge, host timestamp, 10 Hz liveness)
  - `head_d435i_head_link -> ... -> head_d435i_head_depth_optical_frame` (nominal static chain, always available)
- **Fewer misaligned clouds**: Previously, clouds whose TF lookup failed may have been aligned using stale or incorrect poses. With a reliable chain, every cloud is correctly positioned in the fused TSDF volume.
- **Spatial gating works correctly**: The keyframe buffer's spatial gate (translation/rotation difference threshold) uses the camera pose from the TF tree. When the chain was DISCONNECTED, the gate could not verify spatial separation reliably — now it can.

### Diagnostic checks for the next iteration

1. **`marker_map -> head_d435i_head_depth_optical_frame` and `marker_map -> arm_d435i_arm_depth_optical_frame`** should change from "DISCONNECTED entire run" to **CONNECTED for the majority of blocks** (some blocks may still show DISCONNECTED during GTSAM graph reset windows, ~2 blocks out of 35).
2. **`TF chain disconnected` count** should drop from **17** to **<5** (residual disconnections from graph reset windows).
3. **GTSAM output Hz, TF Hz, keyframe_buffer diagnostics** — all unchanged: 14.3 Hz, 50.7 Hz, 1.0 Hz.
4. **TF jump count, odom jumps suppressed, gtsam odometry rejected** — unchanged from current run (3486, 81, 2).
5. **No regression in unit tests**: `gtsam_tracker` has 8 failures (ROS2 env issue), `tsdf_fusion` has 1 failure (same env issue), `keyframe_buffer` 0 failures. These are pre-existing and unrelated.

### Risks

1. **TF timestamp no longer matches pose timestamp**: Downstream nodes that consume both the TF tree and the `/gtsam/head_pose` topic and expect identical timestamps might see a discrepancy. However, TF consumers (pointcloud_fusion, keyframe_buffer, rviz) use `lookup_transform(target, source, Time(0))` — the latest available transform — which is unaffected by the specific timestamp value. Pose topic consumers use the sensor stamp for temporal alignment with sensor data, which is unchanged.
2. **Graph reset intervals still produce brief DISCONNECTED windows**: The scheduled ISAM2 resets at ~48 s intervals clear the graph, causing a ~1-cycle gap in GTSAM TF publishing. During this window (~70 ms), the TF chain reverts to the relay's output (which remains silent because `publish_dynamic_tf=false`). This produces ~2 DISCONNECTED blocks per 35 over a 170 s run. A future iteration could mitigate this by adding a brief relay fallback during reset windows.
3. **No change to clock offset**: The 56460 s chrony offset persists. This fix works around the symptom (TF chain composition failure) without addressing the root cause (un-synchronized Jetson clock). A chrony fix on the Jetson would eliminate the offset entirely but requires hardware access.
