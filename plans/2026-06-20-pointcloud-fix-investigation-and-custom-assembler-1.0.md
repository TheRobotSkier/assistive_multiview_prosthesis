# Pointcloud Fix: Issue Investigation + Custom Assembler

## Objective

Resolve the 100% pointcloud drop rate by (1) fixing confirmed bugs in the Jetson relay gating logic (Issues A & D), (2) confirming the `slop` parameter situation (Issue C), and (3) replacing the failing `depth_image_proc::PointCloudXyzrgbNode` with a custom NumPy-based pointcloud assembler that bypasses all multi-topic time synchronization.

---

## Investigation Findings

### Issue A: Cascaded Gates — CONFIRMED BUG

**Location:** `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/jetson_relay.py:719-734`

**Root Cause:** The `_on_depth` callback applies two gates in sequence:
1. `RateGate` (`depth_{camera}`) — wall-clock throttle (line 731)
2. Token gate — timestamp matching (line 733)

The `RateGate.should_publish()` method (`jetson_relay.py:173-178`) updates `self._last = now` **as a side effect of returning True**. When a depth frame passes the rate limiter but then fails the token check, the rate budget slot is permanently consumed. The next depth frame — which might have matched the token — is blocked by the rate gate because insufficient wall-clock time has elapsed.

**Evidence from report:** Depth arrives at 1.3–1.5 Hz despite `depth.hz` being configured at 5.0+ Hz. The token window is 100ms, but RealSense depth-colour timestamp offsets range 15–80ms. Frames that pass the rate gate but miss the token waste the slot, halving (or worse) the effective depth throughput.

**Fix:** Swap the gate order — check the token gate FIRST, then the rate gate. This way, only frames that match the token consume a rate budget slot.

---

### Issue B: Incompatible Time Domains — CONFIRMED (but secondary)

**Finding:** The report shows `peak |clock_off| = 5.270s`, but the sysmon section clarifies: `true chrony drift is only 0.1ms`. The 5.27s offset is NOT clock skew between Jetson and host — it is the difference between the RealSense ASIC hardware clock (used for image/depth timestamps) and the system clock (used for camera_info and odom). The relay preserves original hardware timestamps (`jetson_relay.py:737-740`), which is correct for depth-colour pairing but means camera_info timestamps (if stamped with system time) are 5+ seconds away from image timestamps.

This makes the 4-way `ApproximateTimeSynchronizer` in `depth_image_proc::PointCloudXyzrgbNode` mathematically unable to match a tuple. However, this issue is **moot** once the custom assembler replaces `depth_image_proc` — the custom node ignores camera_info timestamps entirely.

---

### Issue C: Slop Parameter — CONFIRMED: LIKELY NON-FUNCTIONAL

**Location:** `src/prosthesis_launch/launch/pipeline.launch.py:437-441` and `:455-459`

**Finding:** The launch file passes `"slop": 0.15` to `depth_image_proc::PointCloudXyzrgbNode`. In ROS2 Jazzy's `depth_image_proc` C++ implementation, the `ApproximateTimeSynchronizer` policy does not expose `slop` as a declarable ROS2 parameter in the same way as ROS1. The synchronizer's inter-message interval is controlled at construction time via the message_filters policy, not via runtime parameter loading. If the node does not explicitly declare and read a `slop` parameter, it is silently ignored by the ROS2 parameter system.

Even if `slop` were functional, the 5.27-second time domain split (Issue B) means no reasonable slop value could bridge the gap. This issue is **moot** once the custom assembler replaces `depth_image_proc`.

---

### Issue D: Relay Hz Config Not Passed Down — CONFIRMED BUG

**Location:** `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py:329-340, 467-471`

**Root Cause:** The `relay_hz` master override mechanism is defeated by a non-empty default value on the `relay_img_hz` launch argument.

The flow:
1. Makefile passes `relay_hz:=15.0` (confirmed at `multiview_prosthesis-jetson_docker/Makefile:39`)
2. `_setup()` reads it: `relay_hz_raw = "15.0"` (line 329) — correct
3. Sets `img_default = "15.0"` (line 332) — correct
4. But then: `relay_img_hz = _arg_or_config(context, "relay_img_hz", img_default)` (line 338-340)
5. The `_arg_or_config` function (line 20-22) checks `LaunchConfiguration("relay_img_hz").perform(context)`
6. Since `relay_img_hz` is declared with `default_value="5.0"` (line 469), its value is `"5.0"`, not `""`
7. `_arg_or_config` returns `"5.0"` (because `"5.0" != ""`)
8. The master override `"15.0"` is **silently discarded**

**Impact:** `image.hz` and `depth.hz` are both set to 5.0 instead of the intended 15.0. This limits the depth and image relay to 5 Hz instead of 15 Hz, reducing pointcloud reconstruction opportunities.

**Fix:** Change the `relay_img_hz` launch argument default from `"5.0"` to `""` (empty string), so the master override flows through correctly. The same pattern should be audited for `relay_trackhist_hz` (already empty — OK) and `relay_pc_hz` (already empty — OK, but note line 91 overwrites it with config value; the final assignment on line 335-337 is correct).

---

## Implementation Plan

### Phase 1: Fix Jetson Relay Bugs (Issues A & D)

- [ ] **1.1. Fix cascaded gate order in `_on_depth` (Issue A)**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/scripts/jetson_relay.py`
  - In `_on_depth` (line 719-743): swap the gate order so the token check runs BEFORE the RateGate check. This ensures only token-matching frames consume rate budget.
  - Rationale: A depth frame that fails the token check should not "use up" a rate slot that could have been consumed by the next token-matching frame.

- [ ] **1.2. Fix `relay_img_hz` default value (Issue D)**
  - File: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/launch/dynamic_id2_arm_update_live.launch.py`
  - Change `relay_img_hz` DeclareLaunchArgument `default_value` from `"5.0"` to `""` (line 469).
  - Rationale: The `_arg_or_config` pattern uses empty string as "not set" sentinel. A non-empty default defeats the master override mechanism.

- [ ] **1.3. Remove dead code on line 91 of dynamic_id2_arm_update_live.launch.py**
  - Line 91 (`relay_pc_hz = launch_cfg.get("relay_pc_hz") or pointcloud_max_rate_hz`) overwrites the line 90 computation and is itself overwritten by line 335-337. It is dead code that could confuse future readers. Comment it out or remove it.

### Phase 2: Build Custom Pointcloud Assembler (Replaces depth_image_proc)

- [ ] **2.1. Create `naive_pointcloud_assembler.py` in the camera package**
  - File: `src/camera/camera/naive_pointcloud_assembler.py`
  - Design: A single rclpy Node that subscribes to decompressed depth (`/local/{side}/depth_raw`), RGB (`/local/{side}/image_raw`), and CameraInfo (`/local/{side}/camera_info`).
  - **No time synchronization.** The node caches the latest of each input independently:
    - `_latest_depth`: dict[side, Image] — updated on every depth callback
    - `_latest_rgb`: dict[side, Image] — updated on every RGB callback
    - `_latest_camera_info`: dict[side, CameraInfo] — updated on every camera_info callback (timestamp ignored)
  - **Processing trigger:** A timer at configurable rate (default 5 Hz) checks if new depth is available since last processing. If so, it takes the latest RGB and camera_info and constructs a pointcloud.
    - Alternatively: trigger on depth callback directly, but use a `threading.Lock` + `_processing` flag to skip frames that arrive during processing.
  - **Frame-drop resilience:** If a new depth frame arrives while processing is in progress, it simply overwrites the cached depth. The next timer tick picks it up. No queue, no backpressure.

- [ ] **2.2. Implement depth-to-XYZ deprojection using NumPy**
  - For each pixel (u, v) with depth Z:
    - `X = (u - cx) * Z / fx`
    - `Y = (v - cy) * Z / fy`
    - `Z = Z` (in metres, after dividing by 1000 if depth is in mm)
  - Vectorized with NumPy: create meshgrid of pixel coordinates, multiply by depth array, apply intrinsics.
  - Filter out zero-depth pixels (Z == 0 means no return).
  - Filter out NaN/Inf values.
  - Intrinsics (fx, fy, cx, cy) extracted from cached CameraInfo K matrix.

- [ ] **2.3. Implement RGB color assignment**
  - The depth image is `aligned_depth_to_color` (confirmed in `jetson_relay.py:120-121`), so depth and RGB pixels map 1:1 without an additional transform.
  - Simply read the RGB pixel at the same (u, v) coordinate as the depth pixel.
  - Pack as 0x00RRGGBB float32 (matching PointCloud2 rgb field convention).
  - If RGB image dimensions don't match depth (shouldn't happen with aligned depth, but defensively): resize or crop RGB to match depth dimensions.

- [ ] **2.4. Build and publish PointCloud2 message**
  - Use the same `_build_cloud` pattern as `pointcloud_fusion_node.py:77-104` (4 fields: x, y, z, rgb; point_step=16).
  - Set `header.frame_id` to the camera's depth optical frame (e.g., `head_d435i_head_depth_optical_frame`).
  - Set `header.stamp` to the depth image's original timestamp (preserves the ASIC clock domain for downstream TF lookups via the fusion node's `use_header_stamp_age` mechanism).
  - Publish to `/jetson/{side}/points` (same topic the fusion node subscribes to).

- [ ] **2.5. Register the node in setup.py**
  - File: `src/camera/setup.py`
  - Add entry point: `"naive_pointcloud_assembler = camera.naive_pointcloud_assembler:main"`

- [ ] **2.6. Replace depth_image_proc nodes in pipeline.launch.py**
  - File: `src/prosthesis_launch/launch/pipeline.launch.py`
  - Replace the two `depth_image_proc::PointCloudXyzrgbNode` Node declarations (lines 426-461) with two `naive_pointcloud_assembler` Node declarations.
  - Parameters per node: `side` (head/arm), `depth_topic`, `rgb_topic`, `camera_info_topic`, `output_topic`, `max_rate_hz`.
  - The decompress bridges and camera_info bridges remain unchanged — they still feed `/local/*` topics.

### Phase 3: Cleanup and Verification

- [ ] **3.1. Remove `slop` parameter from launch file**
  - Since `depth_image_proc` nodes are replaced, the `slop: 0.15` parameter is no longer relevant. Remove it to avoid confusion.

- [ ] **3.2. Add diagnostic telemetry to the assembler**
  - Log periodic stats: depth_input_hz, rgb_input_hz, camera_info_received (bool), output_hz, points_per_cloud, processing_time_ms.
  - Use the same `[DIAG-PC]` format that `pipeline_diagnostics_node.py` expects so existing analysis tooling works without changes.

- [ ] **3.3. Verify end-to-end with `make run-camera-log-debug`**
  - Run the pipeline, record a bag, and run `make analyze`.
  - Success criteria: `/jetson/head/points` and `/jetson/arm/points` show > 0 Hz in the bag analysis. `/fused_pointcloud` shows > 0 Hz.

---

## Verification Criteria

- [ ] `/jetson/head/points` and `/jetson/arm/points` publish at > 0 Hz (target: matching depth input rate ~1.5–5 Hz)
- [ ] `/fused_pointcloud` publishes at > 0 Hz
- [ ] Depth relay throughput improves after gate order fix (target: closer to configured `depth.hz`)
- [ ] `relay_hz:=15.0` correctly propagates to `image.hz` and `depth.hz` in the relay node's startup log
- [ ] No `[ERROR]` lines from the assembler node in logs
- [ ] Assembler processing time per frame < 50ms (for 320x240 depth at 5 Hz)

---

## Potential Risks and Mitigations

1. **NumPy performance on large depth images**
   - Risk: 320x240 = 76,800 pixels; full vectorized deprojection + PointCloud2 serialization could be slow in Python.
   - Mitigation: The depth is already downscaled 2x on the Jetson (640x480 → 320x240). At 320x240, NumPy vectorized operations complete in < 10ms on modern hardware. If needed, add spatial decimation (skip every Nth pixel).

2. **RGB and depth dimension mismatch**
   - Risk: If downsampling factors differ between depth and RGB, pixel coordinates won't align.
   - Mitigation: The relay uses `aligned_depth_to_color` and applies the same downsample factor. The assembler should defensively check dimensions and log a warning if they mismatch.

3. **Camera_info not yet received when first depth arrives**
   - Risk: On startup, camera_info may not have arrived yet.
   - Mitigation: Skip processing until camera_info is cached. Log a one-time warning. Camera_info arrives at ~17 Hz so the gap is < 1 second.

4. **Jetson-side changes require Docker rebuild**
   - Risk: The Jetson relay and launch file changes are inside the Docker container. `--symlink-install` may not pick up launch file changes without a rebuild.
   - Mitigation: Run `make build` on the Jetson after changes. The relay script is bind-mounted (`/miahand_ws/src/...`) so script changes are picked up without rebuild, but launch file changes may need `colcon build --packages-select sensor_fusion_bringup`.

---

## Alternative Approaches

1. **Fix depth_image_proc synchronization instead of replacing it**
   - Restamp camera_info with the depth frame's timestamp in the camera_info_bridge.
   - Trade-off: Still depends on the fragile 4-way ApproximateTimeSynchronizer. The 5.27s time domain split would need to be addressed by restamping ALL messages to a common clock. More moving parts, more failure modes.

2. **Use ExactTime synchronizer with restamped messages**
   - Restamp depth, RGB, and camera_info to the same host clock time at the decompress bridge.
   - Trade-off: Loses the original ASIC timestamp, which the fusion node uses for TF lookups. Would require changes to the fusion node's `use_header_stamp_age` logic.

3. **C++ implementation of the custom assembler**
   - Trade-off: Better performance but significantly more development effort. The Python NumPy approach is sufficient for 320x240 at 5 Hz and can be optimized later if needed.
