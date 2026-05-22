# Fix Segmentation Timeout + Camera Mount TF Update Lag

## Objective

Resolve two issues observed in `log_laptop_v2.txt`:

1. **Segmentation inference server unreachable** — every inference request to `127.0.0.1:5678` times out (120s connect timeout).
2. **Camera mount TFs appear to update slowly** — the TFs from `publish_camera_mounts.py` use `StaticTransformBroadcaster`, which relies on `/tf_static` DDS latch semantics. The same CycloneDDS unreliability that required the liveness timer workaround in the OpenVINS bridge node is likely affecting the camera mount TFs.

Additionally, the log reveals a **third critical issue** not previously identified: the `camera_mount_tf_publisher` process is **not started at all** in the v2 log. Line 4-12 shows only 9 processes (no `camera_mount_tf_publisher`). Line 44 shows "Pruning box 0 (fallback)" — the same fallback behavior as before the fix. This means the `mounts_config` fix from the previous session was not effective for this run.

---

## Log Evidence

### Issue 1: Segmentation timeout

- Line 182: `Inference request failed: HTTPConnectionPool(host='127.0.0.1', port=5678): Max retries exceeded`
- Lines 186, 190, 198, 245, 249, 260, 271, 283, 287, 292, 303, 317, 325, 333, 339, 344, 350, 355, 358, 361, 367, 370, 377, 381, 382, 389, 393, 398, 406, 408, 412, 417, 421, 428, 436, etc. — continuous timeout failures.
- Every click from twist propagation (lines 119, 127, 131, etc.) triggers an inference request that times out after 120s.

### Issue 2: Camera mount TF publisher not running

- Lines 4-12: Only 9 processes started. No `camera_mount_tf_publisher` process.
- Line 44: `Pruning box 0 (fallback): frame=arm_d435i_arm_depth_frame` — confirms `mounts_config` was empty/falsy.
- Lines 78-79, 97-98, 102-103, 106-107, 112-113, 116-117, 122, 124, etc.: `Cannot look up arm_d435i_arm_depth_frame in marker_map` and `Cannot transform to arm_d435i_arm_depth_frame for bbox removal` — the TF tree is disconnected.

### Issue 3: TF extrapolation errors (consequence of Issue 2)

- Lines 91, 99, 108, 118, 138, 144, 149, 154, 159, 168, 178, 194, 204, 212, 229, 240, 253, 255, 270, 279, 296, 311, 331, 341, 352, 363, 372, 387, 399, 413, 425, 438: `Lookup would require extrapolation into the past` — OpenVINS bridge publishes at 0.2 Hz liveness rate, and without the camera mount TFs bridging the trees, the TF buffer can't maintain a consistent timeline.

---

## Root Cause Analysis

### Root Cause 1: Segmentation container not starting / not accessible

The `make up` target at `Makefile:108`:
```
cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA) up -d prosthesis segmentation-cuda
```

This uses `$(COMPOSE_SEGMENTATION_CUDA)` which is `-f docker-compose.yml -f docker-compose.segmentation.cuda.yml`. The override adds `runtime: nvidia` and GPU device reservations. However, the `segmentation-cuda` service has `profiles: [segmentation-cuda]` in the base compose file. Even though Docker Compose v2 allows explicitly naming a service on the command line to override the profile restriction, there are two failure modes:

1. **No NVIDIA runtime on the host**: If the host doesn't have `nvidia-container-runtime` installed, the container will fail to start or start without GPU access. The inference server would still run (Flask binds `127.0.0.1:5678`), but the model loading may crash if CUDA is expected but unavailable.

2. **Container crash on startup**: The model loading at `inference_server.py:66-68` happens at module import time (before Flask starts). If the weights download fails (line 8-12 of `entrypoint.inference.sh`) or the model fails to load (CUDA OOM, incompatible PyTorch version), the container exits before Flask binds the port.

3. **Weights download failure**: The weights are downloaded at container start from `https://omnomnom.vision.rwth-aachen.de/...`. If this URL is unreachable from the container (network issue, DNS), the entrypoint exits.

The 120-second connect timeout in the error messages confirms the server never starts listening — it's not a slow response, it's a complete connection failure.

### Root Cause 2: Camera mount TF publisher not starting

The v2 log shows the `camera_mount_tf_publisher` is absent. The fix from the previous session changed `pipeline.launch.py:403` to default `mounts_config` to `/prosthesis_ws/src/sensor_fusion_bringup/config/camera_mounts.yaml`. But the log still shows fallback behavior. Possible reasons:

1. **The fix was not deployed**: The user may have run `make camera-test` without rebuilding `prosthesis_launch`. Since `pipeline.launch.py` is a Python launch file, it's copied (not symlinked) to the install directory by `colcon build`. If the user didn't rebuild after the fix, the installed copy still has the old default.

2. **The script path resolution fails**: Even if `mounts_config` is set, the script at `pipeline.launch.py:195-208` tries to find `publish_camera_mounts.py`. The source-tree path (`__file__/../../../src/sensor_fusion_bringup/scripts/...`) doesn't resolve correctly from the installed launch file. The fallback installed-layout path requires `sensor_fusion_bringup` to be rebuilt with the new CMakeLists.txt that installs `scripts/`. If neither path resolves, the publisher is silently skipped.

### Root Cause 3: Static TFs and CycloneDDS latch issue (the "slow update" the user noticed)

Even once the camera mount TF publisher starts, it uses `StaticTransformBroadcaster` (`publish_camera_mounts.py:160`). This publishes on `/tf_static` with `TRANSIENT_LOCAL` durability. CycloneDDS has known issues with `/tf_static` latch delivery to late-joining nodes (this was the exact reason the OpenVINS bridge node has a 0.2 Hz liveness timer at `openvins_realsense_tf_bridge_node.py:409-423`).

The camera mount TFs don't have a liveness re-send mechanism. If a subscriber misses the initial `/tf_static` latch, it will never see those frames until the publisher restarts. This is the "slow update" the user noticed — the TFs appear stale or absent until a coincidental DDS rediscovery.

The fix from `report.md` (signoff-report section 4) already addressed this for the OpenVINS bridge by adding the liveness timer. The same pattern needs to be applied to `publish_camera_mounts.py`.

---

## Implementation Plan

### Phase 1: Fix segmentation container accessibility

- [ ] **1.1** Add a `make segmentation-status` target to `Makefile` that runs `$(DOCKER_CMD) ps --filter name=segmentation` and shows container status, plus a quick health check (`curl -sf http://127.0.0.1:5678/health`). This gives immediate visibility into whether the container is running and the server is listening.
- [ ] **1.2** Add a `make segmentation-logs` target to `Makefile` that shows the last 50 lines of the segmentation container logs. This is essential for diagnosing startup crashes.
- [ ] **1.3** Verify the `make up` target correctly includes the CUDA compose override. The current `Makefile:108` already uses `$(COMPOSE_SEGMENTATION_CUDA)`. Confirm this resolves correctly for the user's container backend.
- [ ] **1.4** In `entrypoint.inference.sh`, add error handling for the weights download: if `wget` fails, log a clear error message and exit with a non-zero code so the container status shows as unhealthy.
- [ ] **1.5** In `inference_server.py`, add a try/except around model loading (lines 66-68). If loading fails, log the error and exit rather than silently crashing. Add a startup banner that prints "READY" after Flask binds, so logs clearly show when the server is up.

### Phase 2: Fix camera mount TF publisher not starting

- [ ] **2.1** Verify the `prosthesis_launch` package was rebuilt after the `mounts_config` default change. The `camera-test` target should include a dependency or reminder to rebuild. Add a pre-flight check in `pipeline.launch.py` that logs whether `mounts_config` is set and whether the script was found.
- [ ] **2.2** In `pipeline.launch.py`, add explicit logging when `mounts_config` is truthy but the script file is not found. Currently, if both path resolution attempts fail, the publisher is silently skipped (line 209: `if mounts_script and os.path.isfile(mounts_script)`). Add an `else` branch that logs a warning.
- [ ] **2.3** In `Makefile.workspace`, update the `camera-test` target to include a build step for `prosthesis_launch` and `sensor_fusion_bringup` before launching. This ensures the latest code is always installed.

### Phase 3: Fix camera mount TF "slow update" (CycloneDDS latch issue)

- [ ] **3.1** In `publish_camera_mounts.py`, add a dynamic `TransformBroadcaster` alongside the existing `StaticTransformBroadcaster`. Use the same pattern as `openvins_realsense_tf_bridge_node.py`: publish once via `/tf_static`, then re-send on `/tf` at a configurable rate (default 1 Hz).
- [ ] **3.2** Add a `_liveness_tick` method to `CameraMountTFPublisher` that re-sends all camera mount TFs on `/tf` (not `/tf_static`). This ensures late-joining nodes receive the transforms even if they missed the DDS latch.
- [ ] **3.3** Store the list of published TFs in `self._tfs` so the liveness timer can re-send them with updated timestamps.

### Phase 4: End-to-end verification

- [ ] **4.1** After rebuilding, run `make camera-test` and verify:
  - 10 processes started (including `camera_mount_tf_publisher`)
  - Fusion shows 2 pruning boxes (not fallback)
  - No "Cannot look up" or "not part of the same tree" warnings
  - No "extrapolation into the past" TF errors
- [ ] **4.2** Verify segmentation: run `make segmentation-status` and `make segmentation-logs` to confirm the inference server is running. If not, diagnose from the logs.
- [ ] **4.3** Verify TF liveness: restart the fusion node after the camera mount publisher has been running for 30+ seconds. The fusion node should immediately see the camera mount TFs (not wait for a DDS redelivery).

---

## Verification Criteria

1. `make camera-test` starts 10+ processes including `camera_mount_tf_publisher`
2. Fusion log shows `Pruning box 0: frame=palm_frame` and `Pruning box 1: frame=d435i_arm_bottom_screw_frame_8_cm_cam_mount` (not fallback)
3. No `Cannot look up arm_d435i_arm_depth_frame in marker_map` warnings
4. No `not part of the same tree` TF errors
5. No `extrapolation into the past` TF errors (or drastically reduced frequency)
6. `curl http://127.0.0.1:5678/health` returns `{"status": "ok", ...}`
7. Segmentation inference requests succeed (no timeout errors)
8. Twist propagation hit coordinates are in the expected workspace range

---

## Potential Risks and Mitigations

1. **Segmentation container crashes due to CUDA/GPU issue**
   - Mitigation: The health check and `make segmentation-logs` target provide immediate visibility. The inference server already falls back to CPU if CUDA is unavailable. If the host has no NVIDIA GPU, the user can use `segmentation-cpu` instead.

2. **Camera mount TF publisher script path not found in installed layout**
   - Mitigation: The `sensor_fusion_bringup` CMakeLists.txt now installs `scripts/`. The fallback path at `pipeline.launch.py:199-208` uses `ament_index_python` to find the installed share directory. If both fail, the new warning log (Phase 2, step 2.2) makes the issue visible.

3. **Liveness timer adds CPU/network overhead**
   - Mitigation: 1 Hz re-send of ~8 static transforms is negligible overhead. The OpenVINS bridge already does this at 0.2 Hz for 20 transforms.

4. **Rebuilding packages in the container may fail if colcon cache is stale**
   - Mitigation: The `camera-test` target can use `--packages-select` to rebuild only the two affected packages, minimizing build time.

---

## Alternative Approaches

1. **Use `robot_state_publisher` instead of custom script**: Replace `publish_camera_mounts.py` with a URDF/xacro that defines the camera mount frames. `robot_state_publisher` handles `/tf_static` and liveness natively. Trade-off: requires converting YAML transforms to URDF, which is more complex to maintain alongside the existing YAML config.

2. **Use `tf2_ros::StaticTransformBroadcaster` from C++**: A C++ node would handle the DDS latch more reliably than Python. Trade-off: adds build complexity and doesn't solve the fundamental CycloneDDS latch issue — still needs a liveness timer.

3. **Configure CycloneDDS to use reliable delivery for `/tf_static`**: Add QoS overrides in the CycloneDDS XML config. Trade-off: may affect other topics, harder to debug if it doesn't work.
