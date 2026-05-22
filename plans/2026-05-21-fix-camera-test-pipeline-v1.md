# Fix camera-test Pipeline: Missing TFs, Segmentation, Pruning, and Twist Propagation

## Objective

Fix the four interrelated failures observed when running `make camera-test` in the container, as evidenced by `log_laptop.txt`. All issues trace back to missing camera mount TF configuration and a dead segmentation inference server.

## Log Evidence Summary

The log (`log_laptop.txt`) confirms the following sequence of failures:

1. **No camera_mount_tf_publisher process started** — the log shows only 9 processes (lines 4-12), and `camera_mount_tf_publisher` is not among them. This confirms `mounts_config` was empty.

2. **Fusion uses fallback pruning** — line 44: `Pruning box 0 (fallback): frame=arm_d435i_arm_depth_frame` — only 1 pruning box instead of the expected 2 from `camera_mounts.yaml`.

3. **TF tree is disconnected** — line 56: `Could not find a connection between 'marker_map' and 'head_d435i_head_depth_optical_frame' because they are not part of the same tree.` The OpenVINS bridge publishes `head_cam0->head_d435i_head_link` and `arm_cam0->arm_d435i_arm_link` (line 40-42), but without the camera mount TFs, `arm_d435i_arm_depth_frame` is unreachable from `marker_map`.

4. **arm_d435i_arm_depth_frame permanently unreachable** — repeated throughout the log (lines 78-79, 97-98, 102-103, 106-107, 112-113, 116-117, 122-124, 139-140, etc.): `Cannot look up arm_d435i_arm_depth_frame in marker_map for distance filter — skipping` and `Cannot transform to arm_d435i_arm_depth_frame for bbox removal — skipping`. This means distance filtering and hand removal are completely non-functional.

5. **Segmentation inference server is down** — line 182 onwards: `Inference request failed: HTTPConnectionPool(host='127.0.0.1', port=5678): Max retries exceeded`. Every single inference request times out. The twist propagation finds hits and sends clicks, but segmentation can never complete.

6. **Twist propagation fires but is uncontrolled** — lines 119, 127, 131, 136, etc.: hits are found and clicks sent, but segmentation always times out (lines 126, 130, 135, etc.). Without `grasp_contact_frame`, the propagation starts from the camera origin, not the fingertips.

## Root Cause Analysis

### Root Cause 1: `mounts_config` defaults to empty string

`pipeline.launch.py:401-404` declares `mounts_config` with `default_value=""`. The `camera-test` target in `Makefile.workspace:87-89` does not override it. The guard at `pipeline.launch.py:194` (`if mounts_config:`) evaluates to `False`, so `publish_camera_mounts.py` is never started.

**Consequences:**
- No `palm_frame`, `grasp_contact_frame`, `bb_corner`, `bb_opposite`, `d435i_arm_bottom_screw_frame_*`, `bbcam1_frame`, `bbcam2_frame` TFs
- Fusion node falls back to single hardcoded pruning box (confirmed at log line 44)
- `arm_d435i_arm_depth_frame` is unreachable from `marker_map` (confirmed at log lines 56, 78-79, etc.)
- Distance filter and bbox removal are permanently skipped

### Root Cause 2: Segmentation inference server not reachable

The inference server at `http://127.0.0.1:5678` is timing out on every request (log line 182 onwards). The `make up` command starts both `prosthesis` and `segmentation-cuda` containers, but the segmentation container may have crashed, failed to start, or not have its model weights. This needs investigation.

### Root Cause 3: `propagation_origin_offset` is `[0, 0, 0]`

`prosthesis_config.yaml:203` has `propagation_origin_offset: [0.0, 0.0, 0.0]`, so twist propagation starts from the arm camera's tracked position, not the fingertips. The `grasp_contact_frame` TF that defines this offset (published by `publish_camera_mounts.py`) doesn't exist because of Root Cause 1.

### Root Cause 4: `sensor_fusion_bringup` CMakeLists.txt doesn't install scripts

`sensor_fusion_bringup/CMakeLists.txt:6-14` only installs `config/` and `launch/`. The `scripts/` directory is not installed, so the fallback installed-layout path in `pipeline.launch.py:199-208` would also fail.

## Implementation Plan

### Phase 1: Fix camera mount TFs (Issues 1, 3, 4)

- [ ] **1.1** Set `mounts_config` default in `pipeline.launch.py` to auto-detect the config path. Change the `DeclareLaunchArgument` for `mounts_config` (line 401-404) to default to the source-tree path `/prosthesis_ws/src/sensor_fusion_bringup/config/camera_mounts.yaml` with a fallback to the installed share path. Rationale: The dev container bind-mounts `src/`, so the source-tree path always exists. This ensures the camera mount TF publisher starts by default.

- [ ] **1.2** Install `scripts/` in `sensor_fusion_bringup/CMakeLists.txt`. Add `install(DIRECTORY scripts/ DESTINATION share/${PROJECT_NAME}/scripts USE_SOURCE_PERMISSIONS)` after the existing config install. Rationale: The fallback installed-layout path in `pipeline.launch.py:205-206` looks for `share/sensor_fusion_bringup/scripts/publish_camera_mounts.py`, but scripts are never installed.

- [ ] **1.3** Update `Makefile.workspace` `camera-test` target to explicitly pass `mounts_config`. Add `mounts_config:=/prosthesis_ws/src/sensor_fusion_bringup/config/camera_mounts.yaml` to the launch command. Rationale: Belt-and-suspenders — ensures the config path is always correct for this target regardless of the default.

- [ ] **1.4** Set `propagation_origin_offset` in `prosthesis_config.yaml` to the correct camera-to-grasp-contact offset for the `8_cm_cam_mount`. Compute the offset from the mount geometry in `camera_mounts.yaml` (screw_to_link + screw_to_palm transforms) and set it in the `twist_propagation` section. Rationale: The offset from the arm camera optical center to the grasp contact point (fingertips) depends on the mount. Currently `[0,0,0]` means propagation starts at the camera, not the hand.

### Phase 2: Fix segmentation inference server (Issue 2)

- [ ] **2.1** Verify the segmentation-cuda container is running and healthy. Check `docker ps` / `podman ps` for the `segmentation-cuda` container. If not running, check `docker logs segmentation-cuda` for crash reasons. Rationale: Log line 182 shows connection refused to port 5678, meaning the inference server is not listening.

- [ ] **2.2** Check if model weights exist in the `segmentation-weights` volume. The inference server may fail to start if weights are missing. If missing, the weights need to be downloaded or the container needs to be rebuilt. Rationale: The Dockerfile mounts a named volume for weights persistence.

- [ ] **2.3** Add a startup health check or pre-flight log in the segmentation container. Ensure the inference server logs a "ready" message on port 5678 so it's easy to verify it's alive before running the pipeline. Rationale: The current log gives no visibility into whether the inference server started successfully.

### Phase 3: Verify the fix end-to-end

- [ ] **3.1** Rebuild `sensor_fusion_bringup` package: `colcon build --packages-select sensor_fusion_bringup`

- [ ] **3.2** Run `make camera-test` and verify in the log output:
  - `camera_mount_tf_publisher` process appears in the launch output
  - Fusion shows 2 pruning boxes (palm_frame + screw_frame), not fallback
  - No "Cannot look up arm_d435i_arm_depth_frame" warnings
  - No "not part of the same tree" TF errors
  - Segmentation inference server responds on port 5678

- [ ] **3.3** Verify twist propagation uses the correct grasp contact offset by checking hit coordinates are reasonable (near the hand workspace, not at the camera position).

## Verification Criteria

1. `camera_mount_tf_publisher` process appears in launch output (9 → 10 processes)
2. Fusion log shows `Pruning box 0: frame=palm_frame` and `Pruning box 1: frame=d435i_arm_bottom_screw_frame_8_cm_cam_mount` (not fallback)
3. No `Cannot look up arm_d435i_arm_depth_frame in marker_map` warnings
4. No `not part of the same tree` TF errors
5. Segmentation inference returns results (no `Connection timed out` errors on port 5678)
6. Twist propagation hit coordinates are within the grasp workspace bounding box defined in `camera_mounts.yaml`

## Potential Risks and Mitigations

1. **Source-tree path not available in production image**
   Mitigation: The installed-layout fallback in `pipeline.launch.py:199-208` already handles this. Fix 1.2 ensures scripts are installed.

2. **Wrong `propagation_origin_offset` value**
   Mitigation: Compute the offset carefully from the known mount geometry. The `8_cm_cam_mount` has `screw_to_palm` translation `(0.01585, -0.091762, 0.160955)` with quaternion `(-0.5, -0.5, 0.5, -0.5)`. The grasp contact is at `(0, 0.13, 0.03)` in palm_frame. The offset needs to be expressed in the camera (screw) frame by inverting the screw_to_palm transform and applying it to the grasp contact point.

3. **Segmentation container GPU access**
   Mitigation: Ensure NVIDIA Container Toolkit is installed and the GPU override compose file is applied. Check `docker logs segmentation-cuda` for CUDA errors.

4. **TF timestamp extrapolation errors persist**
   Mitigation: The log shows "extrapolation into the past" errors (e.g., line 91). These are caused by the OpenVINS bridge's TF cache being cleared when it reconnects. This is a pre-existing issue unrelated to the camera mount fixes.

## Alternative Approaches

1. **Compute `propagation_origin_offset` dynamically via TF lookup**: Instead of hardcoding the offset in the config, have the twist propagation node look up `grasp_contact_frame` relative to the hand pose frame via TF2 at startup. This would make it mount-agnostic. Trade-off: adds TF dependency to twist propagation startup, but is more maintainable.

2. **Merge camera mount TFs into the OpenVINS bridge node**: Instead of a separate `publish_camera_mounts.py` process, have the bridge node also publish the camera mount static TFs. Trade-off: couples the bridge to mount configuration, but reduces process count.

3. **Use `tonight-safe` launch args for camera-test**: The `TONIGHT_SAFE_LAUNCH_ARGS` already includes the right parameters. Merge `camera-test` into using those args with the `mounts_config` override. Trade-off: may change existing behavior for other developers.
