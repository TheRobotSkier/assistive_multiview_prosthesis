# Fix Camera Mount Quaternion Bugs and Add Pipeline TF Diagnostics

## Objective

Fix two latent geometry bugs in `publish_camera_mounts.py` and add a TF-tree diagnostic monitor to `pipeline.launch.py` so that `make camera-test` produces clear, actionable log output when the TF chain is broken — distinguishing camera-mount issues from OpenVINS/Jetson connectivity issues.

## Latent Bugs Summary

### Bug 1: Standalone root publishes 180° X-rotation instead of identity
**Location**: `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py:270-272`

The standalone-mode root TF `world -> palm_frame` uses quaternion `[1.0, 0.0, 0.0, 0.0]`, which is a **180° rotation about X**. The comment says "identity", but identity is `[0.0, 0.0, 0.0, 1.0]`.

**Impact**: Only affects standalone/RViz visualization mode (`--link-frame=""`). `camera-test` uses pipeline mode, so this is latent.

### Bug 2: `_build_pipeline_root` hardcodes identity-rotation assumption
**Location**: `src/sensor_fusion_bringup/scripts/publish_camera_mounts.py:285-291`

```python
# screw_to_link has identity rotation, so inverse is just negated translation
return [
    _make_tf(s, self._link_frame, screw_frame,
             -_t(sl, "x"), -_t(sl, "y"), -_t(sl, "z"),
             0.0, 0.0, 0.0, 1.0),
]
```

This happens to produce the correct result today because `screw_to_link.quaternion` is identity in the current YAML. However, it **does not use the actual quaternion from the config**, and the helper `_invert_transform()` already exists in the same file. If anyone later edits the YAML to add a non-identity rotation (e.g., to account for mechanical tolerance), the mount geometry will be silently wrong because the rotation is hardcoded to identity.

**Impact**: Latent correctness bug. The translation from the YAML *is* used, but the inversion logic is fragile.

## Implementation Plan

### Phase 1: Fix geometry bugs in `publish_camera_mounts.py`

- [ ] **1.1** Fix standalone root quaternion to identity. In `_build_standalone_root`, change the quaternion arguments from `1.0, 0.0, 0.0, 0.0` to `0.0, 0.0, 0.0, 1.0`.
  - **Rationale**: Matches the documented intent (identity) and prevents a 180° flip in standalone RViz mode.

- [ ] **1.2** Refactor `_build_pipeline_root` to use `_invert_transform()`. Replace the manual negated-translation + hardcoded identity quaternion with:
  ```python
  tx, ty, tz, qx, qy, qz, qw = _invert_transform(sl["translation"], sl["quaternion"])
  return [_make_tf(s, self._link_frame, screw_frame, tx, ty, tz, qx, qy, qz, qw)]
  ```
  - **Rationale**: Makes the inversion robust to future non-identity rotations in the YAML. Reuses the existing, tested `_invert_transform` helper. Keeps the translation from the YAML (which the current code does use), but also correctly inverts the rotation if it ever changes.

- [ ] **1.3** Update the docstring/comment in `_build_pipeline_root` to remove the "identity rotation" assumption and document that the full rigid transform is inverted via `_invert_transform`.
  - **Rationale**: Prevents future developers from re-introducing the hardcoded assumption.

### Phase 2: Add TF pipeline diagnostics

- [ ] **2.1** Create `src/sensor_fusion_bringup/scripts/tf_pipeline_diagnostics.py` — a small rclpy node that:
  - Subscribes to `/tf` and `/tf_static` via a `tf2_ros.Buffer` + `TransformListener`.
  - After a configurable startup delay (default 10 s), periodically checks these frame pairs:
    - `marker_map -> head_d435i_head_depth_optical_frame` (OpenVINS head chain)
    - `marker_map -> arm_d435i_arm_depth_optical_frame` (OpenVINS arm chain)
    - `arm_d435i_arm_link -> palm_frame` (camera mount root)
    - `palm_frame -> grasp_contact_frame` (camera mount internal)
  - Logs **one clear line per check** at `INFO` when OK and `WARN` when missing.
  - If the `marker_map -> *` chains fail but `arm_d435i_arm_link -> palm_frame` succeeds, logs a specific message like:
    ```
    [WARN] OpenVINS TF chain disconnected (marker_map not reachable). Camera mount TFs are OK. Check Jetson bridge connectivity.
    ```
  - If `arm_d435i_arm_link -> palm_frame` fails, logs:
    ```
    [WARN] Camera mount TF chain disconnected. Check that publish_camera_mounts.py is running and mounts_link_frame is correct.
    ```
  - **Rationale**: Gives immediate, actionable feedback during `make camera-test` without requiring manual `tf2_echo` commands.

- [ ] **2.2** Install the new diagnostics script in `sensor_fusion_bringup/CMakeLists.txt`. The existing `install(DIRECTORY scripts/ ...)` already covers it, but verify the new file has executable permissions.
  - **Rationale**: Ensures the installed-layout path in the container works.

- [ ] **2.3** Add `tf_diagnostics` launch argument to `pipeline.launch.py` (default `true`).
  - **Rationale**: Easy to disable in production if the extra logging is unwanted, but on by default for dev/test.

- [ ] **2.4** Conditionally launch the diagnostics node in `pipeline.launch.py` `_launch_setup` (near the camera mount publisher, around line 228). Pass the monitored frame pairs as parameters so they are configurable without editing code.
  - **Rationale**: Integrates the diagnostics into the standard pipeline launch so `make camera-test` automatically gets the debug output.

- [ ] **2.5** Wire `tf_diagnostics:=true` into the `camera-test` target in `Makefile.workspace` (or rely on the new default `true`).
  - **Rationale**: Ensures the diagnostic logging is active during the specific test the user is running.

### Phase 3: Verification

- [ ] **3.1** Run `make camera-test` and inspect the log for the new diagnostic messages.
  - Expect `[INFO] Camera mount TF chain: OK` within ~10 s of launch.
  - If Jetson is not connected, expect the explicit `WARN` that points to OpenVINS rather than camera mounts.

- [ ] **3.2** Verify the `_build_pipeline_root` change by temporarily editing `camera_mounts.yaml` `screw_to_link.quaternion` to a non-identity value (e.g., `[0.0, 0.0, 0.3826834, 0.9238795]` for 45° Z) and running `python3 publish_camera_mounts.py --config ... --link-frame arm_d435i_arm_link --mount 8_cm_cam_mount` in isolation. Check with `tf2_echo` that the screw frame rotation is correctly inverted.
  - **Rationale**: Proves the `_invert_transform` path works for non-identity rotations, whereas the old code would have produced an incorrect result.

- [ ] **3.3** Verify standalone mode fix by running `python3 publish_camera_mounts.py --config ...` (no `--link-frame`) and checking with `tf2_echo world palm_frame` that the transform has identity rotation.
  - **Rationale**: Confirms Bug 1 is fixed.

## Verification Criteria

1. `publish_camera_mounts.py` `_build_standalone_root` outputs quaternion `[0, 0, 0, 1]` for `world -> palm_frame`.
2. `publish_camera_mounts.py` `_build_pipeline_root` calls `_invert_transform(sl["translation"], sl["quaternion"])` and passes all 7 returned values to `_make_tf`.
3. `make camera-test` log contains `[INFO] Camera mount TF chain: OK` (or equivalent) if the camera mount publisher is running.
4. `make camera-test` log contains a clear `WARN` distinguishing OpenVINS disconnect from camera-mount failures.
5. Temporary non-identity rotation in `camera_mounts.yaml` produces a correctly inverted screw frame (verified via `tf2_echo`).

## Potential Risks and Mitigations

1. **Diagnostics node adds startup latency or noise**
   Mitigation: The diagnostics node is lightweight (only does `can_transform` lookups at 0.1 Hz after a 10 s delay). It can be disabled via `tf_diagnostics:=false`.

2. **`_invert_transform` changes behavior for the current identity quaternion**
   Mitigation: For identity `[0, 0, 0, 1]`, `_invert_transform` returns negated translation and `[0, 0, 0, 1]`, which is exactly what the old hardcoded code produced. No behavior change for the current config.

3. **New script missing executable bit in git**
   Mitigation: Run `chmod +x` on the new `.py` file and ensure it is tracked with `git add --chmod=+x` or equivalent.

## Alternative Approaches

1. **Inline diagnostics in `publish_camera_mounts.py`**: Instead of a separate diagnostics node, add a `tf2_ros.Buffer` to the existing `CameraMountTFPublisher` and log the OpenVINS connectivity check on startup. Trade-off: simpler process count, but mixes responsibilities (publisher vs. diagnostics).

2. **Add diagnostics to `pointcloud_fusion_node.py`**: The fusion node already does TF lookups and could log clearer messages. Trade-off: would need to modify C++ code and rebuild; the Python diagnostics script is faster to iterate.

3. **Use `launch_testing` / `pytest` instead of runtime diagnostics**: A formal post-launch test script could verify TF connectivity. Trade-off: heavier infrastructure; runtime log diagnostics give immediate human-readable feedback during normal operation.
