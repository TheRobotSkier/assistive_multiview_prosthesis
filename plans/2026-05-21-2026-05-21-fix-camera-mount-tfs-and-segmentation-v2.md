# Fix Camera Mount TFs + Segmentation Timeout (Revised)

## Objective

Fix two issues from `log_laptop_v2.txt`:
1. **Segmentation inference server unreachable** — every request to `127.0.0.1:5678` times out
2. **Camera mount TFs not starting / unreliable** — the publisher never started in the logged run, and even when it does start, the TFs may be unreliable due to CycloneDDS `/tf_static` latch issues

## Revised Analysis

### The v2 log shows the camera_mount_tf_publisher NEVER started

**Definitive evidence:**
- Lines 4-12: Only 9 processes. No `camera_mount_tf_publisher`.
- Line 44: `Pruning box 0 (fallback): frame=arm_d435i_arm_depth_frame` — this message only appears when `mounts_config` is empty/falsy (see `pointcloud_fusion_node.py:253-258`).
- Line 46: `pruning_boxes=1` — only the fallback box, not the 2 proper boxes from `camera_mounts.yaml`.

**Why the fix didn't take effect:** The source code at `pipeline.launch.py:403` has the correct default (`/prosthesis_ws/src/sensor_fusion_bringup/config/camera_mounts.yaml`). But `colcon build` copies (not symlinks) Python launch files to `install/prosthesis_launch/share/prosthesis_launch/launch/`. The installed copy still had the old `default_value=""` because `prosthesis_launch` was not rebuilt after the fix.

The `camera-test` target at `Makefile.workspace:87-89` does pass `mounts_config:=...` explicitly on the command line, which should override the default. But the launch file processes this in an `OpaqueFunction` — if the installed launch file has a bug in the script path resolution (lines 195-208), the publisher would still be skipped even with `mounts_config` set.

**Why the user saw TFs in RViz:** The user likely saw the 20 static TFs from the OpenVINS bridge (`openvins_realsense_tf_bridge_node.py:318-343` — "Published nominal static chain: 20 transforms"). These include `arm_d435i_arm_link`, `arm_d435i_arm_depth_frame`, etc. — the RealSense camera body frames. The camera mount frames (`palm_frame`, `grasp_contact_frame`, `bb_corner`, `bb_opposite`, `bbcam1_frame`, `bbcam2_frame`) are NOT among these 20 transforms. The user may have seen the camera body TFs and assumed the mount TFs were also present.

Alternatively, the user may have run the publisher manually in a different terminal/session that wasn't captured in this log.

### The "slow update" concern is still valid

Even once the publisher starts, it uses `StaticTransformBroadcaster` (`publish_camera_mounts.py:160`), which publishes on `/tf_static` with `TRANSIENT_LOCAL` durability. This is the exact same CycloneDDS latch unreliability that was fixed in the OpenVINS bridge (see `openvins_realsense_tf_bridge_node.py:411-423` — a 0.2 Hz liveness timer). The camera mount TFs have no such liveness mechanism.

The log shows evidence of intermittent TF connectivity for the arm camera chain:
- Line 74: `published=71, cam1_only=71` — head camera works, arm doesn't
- Line 134: `published=37, dual=29, bbox_removed=339523` — arm camera briefly works!
- Line 143: `bbox_removed=0` — arm camera TF chain lost again
- This intermittent pattern repeats throughout the entire log

This is consistent with the OpenVINS bridge's 0.2 Hz liveness timer (line 43: "switching to liveness mode (0.2 Hz)"). The bridge TFs arrive every 5 seconds, and if the fusion node's cloud timestamp falls between liveness ticks, the TF buffer can't find the transform. The camera mount TFs, once added, will have the same problem unless they also have a liveness mechanism.

### Segmentation container

The `make up` target at `Makefile:108` now includes `$(COMPOSE_SEGMENTATION_CUDA)`. But the segmentation container is still unreachable. The `segmentation-cuda` service has `profiles: [segmentation-cuda]` in `docker-compose.yml:96-97`. When you explicitly name a service on the `docker compose up` command line, Docker Compose v2 should start it regardless of profile. But the container may be crashing on startup (weights download failure, CUDA unavailability, model loading error).

**Diagnosis needed:** Run `docker ps -a --filter name=segmentation` and `docker logs segmentation-cuda` on the host.

---

## Implementation Plan

### Phase 1: Ensure camera mount TF publisher starts reliably

- [ ] **1.1** Rebuild `prosthesis_launch` and `sensor_fusion_bringup` inside the container: `make build-pkg PKG=prosthesis_launch && make build-pkg PKG=sensor_fusion_bringup`. This updates the installed launch file with the correct `mounts_config` default and installs the `scripts/` directory.
- [ ] **1.2** Add a startup log line in `pipeline.launch.py` that explicitly logs whether `mounts_config` is set and whether the publisher script was found. Currently, if the script path resolution fails (lines 195-208), the publisher is silently skipped. Add an `else` branch at line 221 that logs: `[WARN] mounts_config is set but publish_camera_mounts.py not found at <path>`.
- [ ] **1.3** Verify the script path resolution works. The source-tree path at line 195-198 resolves from `__file__` (the installed launch file) via `../../../src/sensor_fusion_bringup/scripts/`. From the installed path `/prosthesis_ws/install/prosthesis_launch/share/prosthesis_launch/launch/pipeline.launch.py`, this goes to `/prosthesis_ws/install/prosthesis_launch/src/sensor_fusion_bringup/scripts/` — which doesn't exist. The fallback at lines 199-208 uses `ament_index_python` to find the installed share directory, which should work after rebuilding `sensor_fusion_bringup`. Test this explicitly.

### Phase 2: Add liveness timer to camera mount TF publisher (fix "slow update")

- [ ] **2.1** In `publish_camera_mounts.py`, add a `TransformBroadcaster` (dynamic) alongside the existing `StaticTransformBroadcaster`. Store all published TFs in `self._tfs`.
- [ ] **2.2** Add a `_liveness_tick` method that re-sends all camera mount TFs on `/tf` (not `/tf_static`) at 1 Hz with updated timestamps. This is the same pattern as `openvins_realsense_tf_bridge_node.py:411-423`.
- [ ] **2.3** Create the liveness timer in `__init__` after the initial `sendTransform` call.

### Phase 3: Diagnose and fix segmentation container

- [ ] **3.1** Add `make segmentation-status` and `make segmentation-logs` targets to `Makefile` for quick diagnostics from the host.
- [ ] **3.2** In `entrypoint.inference.sh`, add error handling: if `wget` fails, print a clear error and exit non-zero.
- [ ] **3.3** In `inference_server.py`, wrap model loading (lines 66-68) in try/except. If loading fails, log the error and `sys.exit(1)` instead of silently crashing.
- [ ] **3.4** Verify the container starts: `docker ps -a --filter name=segmentation` and `docker logs segmentation-cuda` on the host.

### Phase 4: Verification

- [ ] **4.1** After rebuilding, run `make camera-test` and verify:
  - 10 processes started (including `camera_mount_tf_publisher`)
  - Log line: "Published TF tree for mount '8_cm_cam_mount' + bounding box"
  - Fusion shows: `Pruning box 0: frame=palm_frame` and `Pruning box 1: frame=d435i_arm_bottom_screw_frame_8_cm_cam_mount`
  - `pruning_boxes=2` (not 1)
  - No "Cannot look up arm_d435i_arm_depth_frame" warnings (or drastically reduced)
  - No "not part of the same tree" errors
- [ ] **4.2** Verify segmentation: `curl http://127.0.0.1:5678/health` from the host or container.
- [ ] **4.3** Verify TF liveness: after 30+ seconds of running, restart a subscriber node and confirm it immediately sees the camera mount TFs.

---

## Verification Criteria

1. `make camera-test` log shows `camera_mount_tf_publisher` process (10+ processes)
2. Fusion log shows 2 pruning boxes from `camera_mounts.yaml`, not fallback
3. `bbox_removed` counter is consistently non-zero (not intermittently zero)
4. No persistent "Cannot look up arm_d435i_arm_depth_frame" warnings
5. No "not part of the same tree" errors for either camera
6. `curl http://127.0.0.1:5678/health` returns `{"status": "ok"}`
7. Segmentation inference succeeds (no timeout errors)

---

## Potential Risks and Mitigations

1. **Installed launch file not updated after source change**
   - Mitigation: Phase 1 step 1.1 explicitly rebuilds. Phase 1 step 1.2 adds logging so future failures are visible immediately.

2. **Script path resolution fails in installed layout**
   - Mitigation: The `sensor_fusion_bringup` CMakeLists.txt now installs `scripts/`. The `ament_index_python` fallback should work. Phase 1 step 1.3 verifies this.

3. **Liveness timer overhead**
   - Mitigation: 1 Hz re-send of ~8 static transforms is negligible. The OpenVINS bridge already does 0.2 Hz for 20 transforms.

4. **Segmentation container requires GPU**
   - Mitigation: The inference server already falls back to CPU if CUDA is unavailable. If the host has no NVIDIA GPU, use `make segmentation-cpu` instead.

---

## Alternative Approaches

1. **Add a build step to `camera-test`**: Instead of requiring manual rebuild, have `camera-test` run `colcon build --packages-select prosthesis_launch sensor_fusion_bringup` before launching. Trade-off: adds ~10-30 seconds to every `camera-test` run.

2. **Use `--symlink-install` for all builds**: This would make the installed launch file a symlink to the source, so changes take effect immediately. Trade-off: requires changing the build workflow, and some packages don't support symlink install correctly.

3. **Move camera mount TFs into the OpenVINS bridge node**: Instead of a separate script, have the bridge node also publish the camera mount TFs. This eliminates the script path resolution issue entirely. Trade-off: couples the camera mount config to the bridge node, making it harder to maintain independently.
