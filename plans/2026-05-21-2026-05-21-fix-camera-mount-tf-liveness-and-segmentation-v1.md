# Fix Camera Mount TF Liveness and Segmentation Startup

## Objective

Fix two issues observed during `make camera-test`:
1. Camera mount TFs appear to update very slowly (~0.2 Hz visual update in RViz) — the same CycloneDDS `/tf_static` latch unreliability already fixed in the OpenVINS bridge
2. Segmentation inference server never comes online (port 5678 unreachable) — likely weights download on every container restart

## Findings

### Finding 1: Camera mount TFs are static-only — no liveness re-send

**Source**: `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py:160`
```python
self._broadcaster = StaticTransformBroadcaster(self)
```

The publisher uses only `StaticTransformBroadcaster` (publishes once on `/tf_static` with `TRANSIENT_LOCAL` durability). It has a 1 Hz timer (`_republish_marker` at line 187), but that only re-publishes the **Marker** visualization, not the TFs.

The OpenVINS bridge had this exact same problem and was fixed with a two-phase approach (`openvins_realsense_tf_bridge_node.py:256-423`):
- Phase 1: High-rate startup (publishes on both `/tf_static` and `/tf`)
- Phase 2: 0.2 Hz liveness timer re-sends bridge edges on `/tf` only

The camera mount publisher needs the same treatment. Without it, late-joining nodes (or nodes that missed the CycloneDDS latch) never see `palm_frame`, `grasp_contact_frame`, `bbcam1_frame`, etc.

**Important nuance**: The log (`log_laptop_v2.txt`) shows `Pruning box 0 (fallback)` at line 44, meaning the camera mount publisher **did not start** in this particular run. But the user reports seeing the TFs in RViz in other runs. The "slow update" they describe (~0.2 Hz) matches the OpenVINS bridge's liveness timer frequency, suggesting they may have been seeing the OpenVINS bridge's `/tf` re-sends of `arm_cam0 -> arm_d435i_arm_link` rather than the camera mount frames. However, if the camera mount publisher DID start in a different run, the static-only TFs would appear to "jump" or update only when something causes a `/tf_static` re-latch — which would look like ~0.2 Hz or slower.

### Finding 2: Camera mount publisher not starting in logged run

**Source**: `log_laptop_v2.txt:4-12` — only 9 processes, no `camera_mount_tf_publisher`. Line 44 confirms fallback pruning.

**Root cause**: The `prosthesis_launch` package was not rebuilt after the source code change to `pipeline.launch.py:403` (changing `mounts_config` default from `""` to the config path). Launch files are Python files copied to `install/prosthesis_launch/share/prosthesis_launch/launch/` by `colcon build`. Without rebuilding, the installed copy still has the old default.

**Fix**: The `camera-test` target needs to rebuild `prosthesis_launch` and `sensor_fusion_bringup` before launching, or at minimum warn the user.

### Finding 3: Segmentation container — weights not persisted

**Source**: `docker/docker-compose.yml:88` — `segmentation-weights:/weights` is a named Docker volume. `docker/Dockerfile.segmentation:127` — `mkdir -p /weights && chmod 777 /weights`. `entrypoint.inference.sh:8-11` — downloads weights if not found.

**The problem**: The `segmentation-weights` named volume is shared between `segmentation-cuda` and `segmentation-cpu` containers. But more importantly, the user reports "every time I rebuild and restart, it seems like it is just taking forever to come online." This is because:

1. `make build-segmentation-cuda` rebuilds the image (no weights baked in unless `DOWNLOAD_WEIGHTS=1`)
2. `make up` recreates the container
3. The entrypoint downloads ~200MB from `omnomnom.vision.rwth-aachen.de` on every fresh container creation IF the named volume was lost

However, named Docker volumes persist across `docker compose down && up` cycles. They are only lost on `docker compose down --volumes` or `make clean`. So the weights should persist. The "taking forever" is likely:
- First run: downloading 200MB (slow server)
- Subsequent runs: the container starts, loads the model (PyTorch + MinkowskiEngine + CUDA init), which takes 30-60 seconds on first CUDA call
- But the log shows 120-second connection timeouts, meaning the server NEVER binds to port 5678

**The real issue**: Looking at `Makefile:108`:
```makefile
up:
	cd $(COMPOSE_DIR) && $(COMPOSE) $(COMPOSE_SEGMENTATION_CUDA) up -d prosthesis segmentation-cuda
```

This includes `$(COMPOSE_SEGMENTATION_CUDA)` which adds the CUDA override with `runtime: nvidia`. But `segmentation-cuda` has `profiles: [segmentation-cuda]` in the base compose file. In Docker Compose v2, explicitly naming a service on the command line should start it regardless of profile. However, if the user is using **podman-compose** (the auto-detect prefers it), podman-compose may not handle profiles the same way.

Additionally, the inference server at `inference_server.py:66-68` loads the model at **import time** (module level, not inside `if __name__`). If the model loading fails (e.g., CUDA OOM, MinkowskiEngine import error), the process crashes silently — Flask never starts, port 5678 is never bound.

### Finding 4: TF extrapolation errors throughout the log

**Source**: `log_laptop_v2.txt:91,99,108,118,138,144,149,...` — persistent "Lookup would require extrapolation into the past" errors.

These occur because the OpenVINS bridge's liveness timer re-sends bridge TFs at 0.2 Hz (every 5 seconds). The TF buffer has a limited cache window. When clouds arrive with timestamps from the gap between liveness ticks, the lookup fails. This is a direct consequence of the disconnected TF tree — without camera mount TFs bridging `arm_d435i_arm_link` to the mount geometry, the fusion node can't maintain a continuous transform chain.

Once the camera mount TFs are published with a liveness timer (Finding 1), and the publisher starts correctly (Finding 2), these errors should reduce significantly because the TF tree will be continuously connected.

## Implementation Plan

### Phase 1: Fix camera mount TF publisher liveness

- [ ] **1.1** Add `TransformBroadcaster` (dynamic) alongside `StaticTransformBroadcaster` in `publish_camera_mounts.py`
  - Rationale: The OpenVINS bridge uses the same pattern — publish once on `/tf_static` for correct DDS semantics, then re-send on `/tf` at a regular interval for late-joiner reliability
  - Import `TransformBroadcaster` from `tf2_ros` alongside the existing `StaticTransformBroadcaster`
  - Store the TF list as `self._tfs` so the liveness timer can re-send them

- [ ] **1.2** Add a liveness timer that re-sends all camera mount TFs on `/tf` at 1 Hz
  - Rationale: 1 Hz is faster than the OpenVINS bridge's 0.2 Hz, which should provide more reliable TF delivery. The camera mount TFs are truly static (rigid mount), so re-sending them is cheap.
  - Create `self._liveness_timer = self.create_timer(1.0, self._liveness_tick)`
  - `_liveness_tick` updates stamps and calls `self._dynamic_broadcaster.sendTransform(self._tfs)`

- [ ] **1.3** Publish TFs on both `/tf_static` AND `/tf` at startup
  - Rationale: Matches the OpenVINS bridge pattern — immediate availability on both topics
  - In `_publish_single` and `_publish_all`, after `self._broadcaster.sendTransform(tfs)`, also call `self._dynamic_broadcaster.sendTransform(tfs)`

### Phase 2: Ensure camera mount publisher starts

- [ ] **2.1** Add logging in `pipeline.launch.py` when the script path cannot be found
  - Rationale: Currently the publisher is silently skipped when the script isn't found (line 209: `if mounts_script and os.path.isfile(mounts_script)`). Adding a warning log makes debugging easier.
  - Add an `else` branch that logs a warning with the attempted paths

- [ ] **2.2** Update `camera-test` target to rebuild required packages before launching
  - Rationale: The installed launch file was stale, causing the publisher not to start. Adding a build step ensures the installed copy is always current.
  - Add `colcon build --packages-select prosthesis_launch sensor_fusion_bringup` before the launch command in `Makefile.workspace`

### Phase 3: Fix segmentation startup reliability

- [ ] **3.1** Add model-loading error handling and logging to `inference_server.py`
  - Rationale: Currently model loading at lines 66-68 is at module level with no error handling. If it fails, the process exits silently — Flask never starts, port 5678 is never bound.
  - Wrap model loading in try/except with explicit error logging
  - Add a `/health` endpoint readiness flag that only returns "ok" after model is loaded

- [ ] **3.2** Add startup diagnostics to `entrypoint.inference.sh`
  - Rationale: If weights download fails, the container exits silently. Adding explicit logging makes it visible.
  - Add `echo` statements before/after download
  - Add a check that the weights file exists and has non-zero size after download

- [ ] **3.3** Verify weights volume persistence
  - Rationale: The user reports "taking forever to come online" after rebuild+restart. Named volumes persist across `docker compose up/down`, but `make clean` removes them. Need to verify the volume is not being accidentally cleaned.
  - Check if `make up` or any other target calls `down --volumes`
  - The `clean-volumes` target explicitly removes `segmentation-weights`, but `up` and `down` do not

- [ ] **3.4** Add `make segmentation-status` and `make segmentation-logs` targets
  - Rationale: No easy way to check if the segmentation container is running or see its logs from the host.
  - `segmentation-status`: runs `docker ps --filter name=segmentation` 
  - `segmentation-logs`: runs `docker logs -f segmentation-cuda` (or `segmentation-cpu`)

### Phase 4: Verification

- [ ] **4.1** Rebuild `prosthesis_launch` and `sensor_fusion_bringup` inside the container
- [ ] **4.2** Run `make camera-test` and verify:
  - 10 processes started (including `camera_mount_tf_publisher`)
  - Log shows "Pruning box 0: frame=palm_frame" and "Pruning box 1: frame=d435i_arm_bottom_screw_frame_8_cm_cam_mount" (not fallback)
  - No "Cannot look up arm_d435i_arm_depth_frame in marker_map" warnings
  - No "Cannot transform to arm_d435i_arm_depth_frame for bbox removal" warnings
  - TF extrapolation errors are eliminated or significantly reduced
  - `ros2 topic echo /tf --once` shows camera mount frames being published at ~1 Hz
- [ ] **4.3** Verify segmentation:
  - `make segmentation-status` shows container running
  - `make segmentation-logs` shows "[inference_server] Model ready." and "Starting server on http://127.0.0.1:5678"
  - `curl http://127.0.0.1:5678/health` returns `{"status": "ok", ...}`

## Potential Risks and Mitigations

1. **Risk: 1 Hz re-send of static TFs on `/tf` causes TF buffer bloat**
   Mitigation: Static TFs re-sent on `/tf` are small (a few hundred bytes each). At 1 Hz with ~10 TFs, this is negligible. The OpenVINS bridge already does this at 0.2 Hz with 2 TFs.

2. **Risk: Rebuilding packages in `camera-test` adds startup latency**
   Mitigation: `colcon build --packages-select` is incremental and fast for Python-only packages (just copies files). Typical overhead is 2-5 seconds.

3. **Risk: Model loading in inference_server.py fails due to CUDA OOM on laptop GPU**
   Mitigation: The error handling in 3.1 will make this visible. The health endpoint will report the failure clearly.

4. **Risk: Weights download URL is unreachable**
   Mitigation: The entrypoint already checks `if [ ! -f "$WEIGHTS_PATH" ]`. If the volume persists, no download is needed. The diagnostics in 3.2 will make download failures visible.

## Alternative Approaches

1. **Alternative for TF liveness**: Instead of re-sending on `/tf`, configure CycloneDDS with `durability=VOLATILE` for `/tf_static` or increase the latch reliability. This is more invasive and less portable than the re-send approach.

2. **Alternative for segmentation weights**: Bake weights into the Docker image at build time (`DOWNLOAD_WEIGHTS=1`). This eliminates the runtime download but increases image size by ~200MB and requires rebuild to update weights.

3. **Alternative for camera-test rebuild**: Use `--symlink-install` in the build so launch file changes are immediately effective without rebuilding. This is already done in `tonight-build` but not in the regular `build` target.
