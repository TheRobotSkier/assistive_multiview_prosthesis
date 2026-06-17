# Iteration 2 — 2026-06-17

## Goal

Increase the production rate of cross-camera visual factors (`/vis/head_arm_pose`) so the GTSAM factor graph receives frequent head-arm relative-pose constraints, directly improving localization precision over the kinematic-range-only coupling from Iteration 1.

## What was investigated

### Replay results (replay_20260617_021016)

The latest replay shows three STARVED topics; the most critical for localization is **`/vis/head_arm_pose`**:

| Topic | Msgs | Eff Hz | Nom Hz | Health |
|---|---|---|---|---|
| `/gtsam/arm_pose` | 2044 | 11.9 | 15 | OK |
| `/gtsam/head_pose` | 2044 | 11.9 | 15 | OK |
| `/tf` | 5281 | 30.8 | 50 | OK |
| `/keyframe_buffer/diagnostics` | 35 | 0.2 | 1 | STARVED |
| `/tf_static` | 3 | 0.0 | 1 | STARVED |
| **`/vis/head_arm_pose`** | **1** | **0.006** | **30** | **STARVED** |

Key observations:

1. **`/vis/head_arm_pose` has only 1 message in 171 seconds** (should be ≈ 855 at 5 Hz). This means the cross-camera SIFT alignment is producing virtually no visual factors.
2. The GTSAM graph therefore operates with **no cross-chain visual constraints** — only the kinematic range factor (fixed in Iteration 1). The range factor alone is much weaker than combined range + visual factors.
3. The bag metadata confirms no raw sensor topics (images, clouds, camera_info) are present — the replay records pipeline *outputs*, not inputs. The near-zero `/vis/head_arm_pose` count therefore reflects a real deficiency in the SIFT node's ability to produce output during the live pipeline run.

### Root cause: `sync_slop_s` too tight

The `SiftFeatureNode` uses `ApproximateTimeSynchronizer` to pair head + arm images. The synchronizer parameter is:

```python
# sift_feature_node.py:294
"sync_slop_s": 0.05,   # 50 ms time tolerance
```

A 50 ms slop is **very tight** for two independent cameras on a robot arm. In practice:
- Head and arm cameras may have slightly different shutter triggers or processing pipeline delays.
- Network transit jitter (Jetson relay) can push one image stream ahead of the other by >50 ms.
- Clock drift between the two camera sensors can accumulate.

When the synchronizer cannot find a pair within 50 ms, it **drops** the image messages entirely — no callback fires, no SIFT matching runs, no visual factor is published.

This was the intended value from the V6 plan (`plans/2026-06-13-localization-v6-phase3-tsdf-sift.md:211`), but it appears too aggressive for the real sensor timing characteristics.

### Additional observations

- The `keyframe_buffer/diagnostics` topic runs at 0.2 Hz (5 s timer at `keyframe_buffer_node.py:622`) but the analyzer expects 1 Hz — a metadata mismatch, not a functional bug.
- The `process_rate_hz: 5.0` parameter in the SIFT node's `DEFAULT_PARAMS` is not actually used by the node (no timer is created from it). The node is purely callback-driven. This is harmless but worth noting.
- Unit tests: 8 failures in `gtsam_tracker` and 1 in `tsdf_fusion` — all from `rclpy` node-creation errors (`RCLError: error creating node`). This is a test‑infrastructure issue (ROS_DOMAIN_ID / rclpy init) unrelated to pipeline logic.

## Changes made

### 1. `src/cross_camera_features/cross_camera_features/sift_feature_node.py:294`

Increased the ApproximateTimeSynchronizer slop from **50 ms to 200 ms**:

```python
# BEFORE:
"sync_slop_s": 0.05,
# AFTER:
"sync_slop_s": 0.2,
```

### 2. `config/prosthesis_config.yaml:552`

Updated the corresponding runtime parameter override to match:

```yaml
# BEFORE:
sync_slop_s: 0.05
# AFTER:
sync_slop_s: 0.2
```

## Expected impact

**Localization precision (goal 1):**

- The wider 200 ms synchronisation window should allow **many more head–arm image pairs** to be matched by `ApproximateTimeSynchronizer`.
- More matched pairs → more SIFT extractions → more 3‑D‑3‑D correspondences → more `/vis/head_arm_pose` messages published.
- Each visual factor adds a **strong cross-chain geometric constraint** to the GTSAM factor graph, linking the head and arm pose estimates through direct visual alignment rather than just the soft kinematic range factor.
- With frequent visual factors, the smoother should produce **lower trajectory RMS error and less drift** between the two chains.

**TSDF fusion (goal 2, indirect):**

- Better head–arm relative poses from GTSAM mean the poses fed to the TSDF integrator are more consistent.
- This should reduce pointcloud misalignment artefacts and improve the spatial coherence of the fused TSDF volume.

### Quantitative expectation

At 5 Hz process rate with 200 ms slop, we expect:
- `/vis/head_arm_pose` effective rate to rise from **0.006 Hz to >1 Hz** (conservative, depends on scene texture and camera sync quality).
- If the cameras provide ~15 Hz each and are within 200 ms of each other, the synchronizer should produce most pairs, and the node should publish at roughly its processing-limited rate.

## What to check in the next iteration

1. **`/vis/head_arm_pose` effective Hz** in bag_analysis.txt — should be significantly higher.
2. **GTSAM trajectory metrics** — ATE/RPE between odometry input and GTSAM output should improve with more visual factors.
3. **TSDF pointcloud quality** — fused cloud density should increase and outliers decrease.
4. **Cross-camera feature test** — `test_sift_features.py` `match_and_align` tests should still pass (they do not depend on sync_slop).
5. If `/vis/head_arm_pose` remains severely starved after this fix, investigate deeper: check whether the camera image topics are present in the replay bag at all, and whether SIFT matching consistently fails (e.g., low‑texture scene).
