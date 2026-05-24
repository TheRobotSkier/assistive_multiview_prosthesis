# Camera-Test Performance Fixes (Findings 1-4)

## Objective

Fix the four highest-impact performance issues causing TF drops and visual jumping in RViz during camera-test runs.

---

## Finding 1: TF Bridge publishes at 2 Hz with unnecessary timer

**Root cause**: The bridge edge `cam0 -> link` is a **static hardware extrinsic** (never changes). The timer at `openvins_realsense_tf_bridge_node.py:253` exists only to handle:
1. Startup race: waiting for the RealSense static chain to appear in the TF buffer
2. CycloneDDS `/tf_static` latch unreliability for late-joining nodes

**Safety of `/tf` re-publish**: Confirmed safe. Static transforms in tf2's `StaticCache` never expire. The dynamic cache copy expires after 10s but is redundant — the static cache always answers lookups. No extrapolation or expiry risk.

### Implementation

- [ ] **1a.** Refactor `_publish_all` timer callback in `openvins_realsense_tf_bridge_node.py` to use a **two-phase approach**:
  - **Phase 1 (startup)**: Run the timer at ~10 Hz until the bridge extrinsic is successfully resolved for all specs. On each tick, attempt `_resolve_bridge_transform()`. Once all are resolved, publish each on `/tf_static` **once** and transition to Phase 2.
  - **Phase 2 (liveness)**: Replace the fast timer with a slow liveness timer (~0.2 Hz, once every 5 seconds) that re-sends **only the 2 bridge-edge transforms** (not the entire 20-transform nominal chain) on `/tf` for late joiners. Keep the `/tf_static` publish from Phase 1 — do not re-send on `/tf_static`.

- [ ] **1b.** Remove the re-broadcast of the full nominal static chain on `/tf` (lines 355-358). The nominal chain is already sent once on `/tf_static` at startup (line 332). Re-sending 20 transforms on `/tf` every tick is wasteful and floods the dynamic cache.

- [ ] **1c.** Update `config/prosthesis_config.yaml:167` — change `publish_rate_hz` from `2.0` to `10.0` (this now only affects the startup phase duration; the liveness phase is hardcoded slow).

### Verification

- [ ] Bridge publishes bridge-edge transforms on `/tf_static` exactly once after resolution
- [ ] Liveness timer re-sends only bridge edges on `/tf` at ~0.2 Hz
- [ ] No nominal static chain re-broadcast on `/tf`
- [ ] `ros2 topic echo /tf` shows dramatically reduced message frequency after startup

---

## Finding 2: Twist propagation has 500ms blocking TF timeout

**Root cause**: `twist_propagation_node.py:803-806` uses `self._tf_buffer.transform()` with a 0.5s timeout. This is a **blocking call** that freezes the single-threaded executor, preventing TF updates, cloud subscriptions, and other callbacks from being processed. This is the exact same bug documented in the retrospective (Discovery 1) that was fixed in the fusion node and bridge, but was never fixed here.

**Why it's safe to remove the timeout**: The transform being looked up is from `frame_id` to `self._cloud_frame`. Both are frames that should already be in the TF buffer by the time this function is called (the cloud has already been received with its frame_id, and the cloud_frame is set from a previous cloud). A non-blocking lookup with `Time()` (latest available) is sufficient.

### Implementation

- [ ] **2a.** In `twist_propagation_node.py`, replace the blocking `self._tf_buffer.transform()` call at line 803-806 with a non-blocking approach:
  - Use `self._tf_buffer.lookup_transform(self._cloud_frame, frame_id, Time())` (zero timeout, non-blocking)
  - Manually apply the transform to the PointStamped coordinates
  - Fall back to raw coordinates on failure (same as current behavior)

- [ ] **2b.** Add `from rclpy.time import Time` import if not already present.

### Verification

- [ ] No blocking TF calls remain in twist_propagation_node.py
- [ ] Executor no longer stalls when TF is temporarily unavailable
- [ ] Twist propagation still correctly transforms points when TF is available

---

## Finding 3: Cloud max age too high

**Root cause**: `config/prosthesis_config.yaml:145` overrides `cloud_max_age_s` to `1.5`. The code default was `0.5s`. A 1.5-second-old cloud can have a significantly different TF than the current one, causing visual offset/jumping when the fusion node transforms it.

### Implementation

- [ ] **3a.** In `config/prosthesis_config.yaml:145`, change `cloud_max_age_s` from `1.5` to `0.5`.

### Verification

- [ ] Clouds older than 0.5s are dropped by the fusion node
- [ ] Visual jumping is reduced in RViz

---

## Finding 4: Remove pointcloud relay node

**Root cause**: `pointcloud_relay_node.py` subscribes to `/fused_pointcloud` and republishes on `/segmentation/input_cloud`. This is a pure copy — no transformation, no filtering. Every message is serialized, deserialized, and re-published, adding latency and consuming CPU/memory for large point clouds.

**Safety verification**: The `grasp_test.launch.py:109` already bypasses the relay using remapping: `remappings={("/segmentation/input_cloud", cloud_topic)}`. This proves the remapping pattern works in production.

### Implementation

- [ ] **4a.** In `pipeline.launch.py`, add a remapping to the segmentation_bridge node (line 226-231):
  ```
  remappings={("/segmentation/input_cloud", "/fused_pointcloud")}
  ```

- [ ] **4b.** In `pipeline.launch.py`, remove the pointcloud_relay node entry (lines 197-202).

- [ ] **4c.** In `twist_propagation_test.launch.py`, apply the same remapping to the segmentation_bridge node and remove the pointcloud_relay node entry (lines 106-110).

- [ ] **4d.** In `digital_twin.launch.py`, apply the same remapping and remove the pointcloud_relay node entry (lines 183-187).

- [ ] **4e.** Update `scripts/test_pointcloud_health.sh:39` to remove `"pointcloud_relay"` from the node health check list.

- [ ] **4f.** Do NOT modify `segmentation_ros2_node.py` — keep the hardcoded `/segmentation/input_cloud` default. The remapping handles the topic redirect at the launch level.

- [ ] **4g.** Do NOT remove `pointcloud_relay_node.py` or its entry in `setup.py` — keep the file for backward compatibility. Only remove it from active launch files.

### Verification

- [ ] `ros2 node list` does not show `pointcloud_relay` after launch
- [ ] `ros2 topic info /segmentation/input_cloud` shows no subscribers (or the segmentation node subscribes to `/fused_pointcloud` directly)
- [ ] Segmentation still works end-to-end (click → inference → object cloud)
- [ ] `grasp_test.launch.py` is unaffected (already uses remapping)

---

## Risks and Mitigations

1. **CycloneDDS `/tf_static` latch failure**: If late-joining nodes miss both the initial `/tf_static` publish and the slow `/tf` liveness heartbeat, they won't get the bridge transform for up to 5 seconds.
   Mitigation: The 0.2 Hz liveness timer ensures maximum 5-second delay. This is acceptable for development/testing.

2. **Twist propagation non-blocking TF failure**: If the TF is genuinely unavailable, the non-blocking lookup will fail and fall back to raw coordinates. This is the same fallback behavior as the current code, just without the 500ms stall.

3. **Relay removal breaks external tools**: Any tool that depends on `/segmentation/input_cloud` being a separate topic from `/fused_pointcloud` will break.
   Mitigation: All known consumers are in the launch files being updated. The `Makefile.workspace` health check is also updated.

---

## Execution Order

1. Finding 3 (config change, lowest risk, immediate improvement)
2. Finding 1 (bridge refactor, highest impact)
3. Finding 2 (twist propagation fix, code change)
4. Finding 4 (relay removal, multi-file change)
