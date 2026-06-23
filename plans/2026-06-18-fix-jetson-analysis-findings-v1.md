# Fix Jetson Analysis Findings — QoS & Noisy Warnings

## Objective

Apply targeted edits to resolve the two issues discovered by `make analyze-jetson` across 6 Jetson log runs from 2026-06-18:

1. **QoS Incompatibility** — Multiple topics have blocked data flow due to RELIABLE/BEST_EFFORT mismatches between publishers and subscribers.
2. **Noisy Startup Warnings** — `aruco_marker_pose_node.py` emits 26–68 WARN-level messages per run for a condition that resolves itself after OpenVINS initializes.

---

## Summary of Analysis Findings

Across all runs, the `analyze_log.py` QoS audit detected **2 to 9 incompatible topics** per run. The consistent patterns are:

| Topic | Publisher | Subscriber | Mismatch |
|---|---|---|---|
| `/jetson/head/points` | `jetson_relay` (BEST_EFFORT) | external subscriber (RELIABLE) | RELIABILITY |
| `/jetson/arm/points` | `jetson_relay` (BEST_EFFORT) | external subscriber (RELIABLE) | RELIABILITY |
| `/head/marker_pose/observation` | `aruco_marker_pose_node.py` | `run_subscribe_msckf_marker` | RELIABILITY vs RELIABILITY_QOS_POLICY |
| `/arm/marker_pose/observation` | `aruco_marker_pose_node.py` | `run_subscribe_msckf_marker` | RELIABILITY vs RELIABILITY_QOS_POLICY |
| `/arm/marker_pose/dynamic_arm_pose_observation` | `dynamic_arm_pose_measurement_node.py` | `run_subscribe_msckf_marker` | RELIABILITY vs RELIABILITY_QOS_POLICY |
| `/head/marker_pose/dynamic_observation` | `aruco_marker_pose_node.py` | `dynamic_arm_pose_measurement_node.py` | RELIABILITY |

Additionally, the `aruco_marker_pose_node.py` emits **26–68 WARN messages** per run for "No OpenVINS odom received yet" — a self-resolving startup condition.

---

## Fix A: Eliminate Dual-Publisher QoS Collision on Point Cloud Topics

### Root Cause

`pipeline.launch.py` lines 386–414 launch two `depth_image_proc::PointCloudXyzrgbNode` instances that reconstruct colored point clouds locally from decompressed depth/color images. These nodes publish via remapping to `/jetson/head/points` and `/jetson/arm/points` using the **ROS 2 default RELIABLE** QoS.

Simultaneously, the Jetson `jetson_relay` node publishes to these **exact same topics** using **BEST_EFFORT** QoS. This creates a dual-publisher collision where:
- The host `depth_image_proc` node publishes RELIABLE → incompatible with host subscribers using BEST_EFFORT (keyframe_buffer, pointcloud_fusion, pipeline_diagnostics)
- The Jetson relay publishes BEST_EFFORT → compatible with host subscribers
- The two publishers compete on the same topic, generating QoS warnings whenever a new subscriber appears

The comment at line 337–338 states that the `depth_image_proc` path "Replaces Jetson-side pointcloud publishing." But the Jetson relay is still publishing to these topics, creating the collision.

### Exact Edit: Host Side (THIS REPO)

**File:** `src/prosthesis_launch/launch/pipeline.launch.py`

**Change 1 — Line 398:**
- [ ] Replace `("points", "/jetson/head/points")` with `("points", "/local/head/points")`

**Change 2 — Line 411:**
- [ ] Replace `("points", "/jetson/arm/points")` with `("points", "/local/arm/points")`

**Rationale:** This redirects the host-reconstructed point clouds to dedicated `/local/*` topics, eliminating the dual-publisher collision on the `/jetson/*` namespace. No subscriber changes are needed because:
1. The host subscribers (keyframe_buffer, pointcloud_fusion, pipeline_diagnostics) already receive from the Jetson relay's BEST_EFFORT publisher on `/jetson/head/points` and `/jetson/arm/points`.
2. The host `depth_image_proc` local reconstruction was redundant with the Jetson relay's point clouds — both produce the same data. If the host reconstruction is ever needed (e.g., when Jetson relay point cloud publishing is disabled), subscribers can be pointed to `/local/head/points` and `/local/arm/points` via their existing topic parameters.

### Exact Edits: Jetson Side (openvins_overlay — NOT in this repo)

These files are located at `/home/robotlab/openvins_overlay/install_overlay/` on the Jetson. They must be edited directly on the Jetson or included in the overlay rsync package.

**File:** `sensor_fusion_bringup/scripts/aruco_marker_pose_node.py`

- [ ] Locate the "No OpenVINS odom received yet" warning message (search for the exact string in the file)
- [ ] Change the log level from `self.get_logger().warn(...)` to `self.get_logger().debug(...)`
- [ ] OR add a throttle: wrap in a `_warn_count` counter that only emits the first 3 occurrences, then silences

**Rationale:** This warning fires every ~2 seconds from node startup until OpenVINS publishes its first odometry message. On a 60–150 second run, this generates 26–68 WARN lines that obscure actual warnings in the severity tally.

**File:** `openvins/ov_msckf/src/ros/run_subscribe_msckf_marker.cpp` (or wherever the OpenVINS marker subscriber QoS is configured)

- [ ] Change the marker pose subscription QoS from `RELIABILITY_QOS_POLICY` to `RELIABLE` (or `BEST_EFFORT` to match whichever the publishers use)
- [ ] Specifically, the subscriptions to `/head/marker_pose/observation`, `/arm/marker_pose/observation`, `/head/marker_pose/dynamic_observation`, and `/arm/marker_pose/dynamic_arm_pose_observation` must use the same reliability as the publishers (currently `aruco_marker_pose_node.py` and `dynamic_arm_pose_measurement_node.py` publish with RELIABLE as observed from the QoS warnings showing RELIABILITY as the incompatible policy)

**Rationale:** The `run_subscribe_msckf_marker` node reports `RELIABILITY_QOS_POLICY` as its incompatible policy — this indicates it uses a non-standard QoS profile that doesn't match the RELIABLE publishers. Data flow on these topic pairs is zero.

---

## Additional Consideration: Jetson Relay Point Cloud Disable

If the host-side `depth_image_proc` path is truly intended to REPLACE Jetson-side point cloud publishing (per the comment at `pipeline.launch.py:337`), the Jetson relay should also be configured to not publish point clouds:

**File:** Jetson launch configuration (wherever `jetson_relay` parameters are set)

- [ ] Set `pointcloud: enabled=False` in the jetson_relay configuration when the host-side reconstruction is active

However, this is optional. The host-side edit (changing the topic remapping) already eliminates the QoS collision regardless of whether the Jetson relay publishes.

---

## Verification Criteria

- [ ] After applying change to `pipeline.launch.py`, launch the pipeline and run `make analyze-jetson` again — the QoS audit should show zero hits for `/jetson/head/points` and `/jetson/arm/points`
- [ ] No `[WARN]` lines from `aruco_marker_pose_node.py` should appear in the severity tally for a fresh Jetson run
- [ ] After fixing `run_subscribe_msckf_marker` QoS, the `/head/marker_pose/observation` and related topics should show zero QoS hits
- [ ] VIO marker corrections yield remains non-zero (confirming data actually flows after QoS fix)

---

## Potential Risks and Mitigations

1. **Host reconstruction path becomes unreachable**
   - Risk: No subscriber currently reads `/local/head/points` or `/local/arm/points`
   - Mitigation: This is the current de facto state anyway — the RELIABLE publisher was already incompatible with BEST_EFFORT subscribers, so the host reconstruction's clouds were never delivered to any subscriber. The change is a no-op for data flow.

2. **Jetson-side edits require access to the openvins_overlay**
   - Risk: The overlay files are rsynced separately and not in version control with this repo
   - Mitigation: Edits must be made directly on the Jetson or the overlay source. Document the changes so they can be reapplied after overlay updates.

3. **Changing run_subscribe_msckf_marker QoS may affect OpenVINS behavior**
   - Risk: OpenVINS may rely on specific QoS for its internal timing
   - Mitigation: The change is from a non-standard `RELIABILITY_QOS_POLICY` to standard `RELIABLE` — this should only improve compatibility. Test with a short Jetson run to verify VIO still initializes correctly.

---

## Alternative Approaches

1. **Align QoS instead of redirecting topics**: Change the host `depth_image_proc` nodes to use BEST_EFFORT (the nodes are standard ROS2 packages and cannot be reconfigured via launch file — would require custom wrapper nodes or source patches). Not recommended.

2. **Disable Jetson relay point cloud publishing via configuration**: Change the Jetson relay config so `pointcloud: enabled=False` when host reconstruction is active. This avoids the dual-publisher problem entirely but requires coordinating host and Jetson configurations.

3. **Switch all subscribers to RELIABLE**: Change keyframe_buffer, pointcloud_fusion, and pipeline_diagnostics to use RELIABLE instead of BEST_EFFORT. This would make them compatible with the host `depth_image_proc` publisher but incompatible with the Jetson relay's BEST_EFFORT publisher. Not recommended — BEST_EFFORT is preferred for high-throughput sensor data.
