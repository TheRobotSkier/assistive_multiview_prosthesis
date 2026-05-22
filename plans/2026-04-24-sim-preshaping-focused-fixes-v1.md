# Sim ↔ Preshaping Code Drift — Focused Remediation Plan

## Objective

Fix the critical wrist camera frame mismatch that breaks preshaping TF lookups, and clean up the two documentation drift items (stale CLI instructions and phantom topic) that mislead developers. This plan intentionally defers the covariance bridging, planner GUI twist, example verification, and topic namespacing items.

---

## Scope

| Finding | Severity | Included |
|---|---|---|
| 1. Wrist camera TF frame name mismatch | CRITICAL | Yes |
| 2. Covariance drift (Rust ignores C++ values) | HIGH | No — deferred |
| 3. Planner GUI zero twist | HIGH | No — deferred |
| 4. README documents removed Rust CLI | MEDIUM | Yes |
| 5. README documents nonexistent `grasp_start` topic | MEDIUM | Yes |
| 6. Unverified examples warning | LOW | No — deferred |
| 7. Un-namespaced `/hand_pose`, `/hand_twist` | LOW | No — deferred |

---

## Implementation Plan

### Phase 1: Wrist Camera Frame Name Fix (P1 — Critical)

The root cause: three different frame IDs are used for the same wrist camera across the codebase.

| Component | Frame ID used | File |
|---|---|---|
| TF publisher (scene state) | `mujoco_camera_wrist_cam` | `mujoco_scene_state_publisher_node.py:310` — auto-generated from MuJoCo camera name `wrist_cam` via `mujoco_camera_{sanitize(name)}` |
| IMU publisher (interactive) | `mujoco_wrist_cam` | `interactive_system_interface.cpp:543,563` |
| Preshaping bridge TF lookup | `mujoco_camera_wrist_cam` | `preshaping_service_bridge_node.cpp:57` |
| README documentation | `mujoco_wrist_cam` | `README.md:107` |

The TF tree publishes `mujoco_camera_wrist_cam`. The preshaping bridge's default `camera_frames_` list already matches this. The IMU publisher, however, uses `mujoco_wrist_cam` — a different frame ID. Any node that tries to cross-reference IMU data with TF will fail.

- [x] **Task 1.1**: Update the IMU frame IDs in `interactive_system_interface.cpp` to match the TF-published frame name.
  - Change `interactive_system_interface.cpp:543` from `"mujoco_wrist_cam"` to `"mujoco_camera_wrist_cam"`.
  - Change `interactive_system_interface.cpp:563` from `"mujoco_wrist_cam"` to `"mujoco_camera_wrist_cam"`.
  - **Rationale**: The TF tree is the authoritative source of frame names. All consumers (IMU, preshaping bridge, RViz) should use the TF-published name.

- [x] **Task 1.2**: Update the README to reflect the correct wrist camera frame name.
  - Change `README.md:107` from `"mujoco_wrist_cam"` to `"mujoco_camera_wrist_cam"`.
  - **Rationale**: Documentation must match the actual frame IDs used in code.

- [x] **Task 1.3**: Verify the preshaping bridge's default `camera_frames_` parameter is correct.
  - Confirm that `preshaping_service_bridge_node.cpp:56-57` lists `"mujoco_camera_wrist_cam"` (it already does — no change needed, just verify).
  - **Rationale**: Ensures the TF lookup in `try_handle_direct_request` will succeed for the wrist camera.

- [x] **Task 1.4**: Build and run a smoke test.
  - Launch the interactive simulator with `scene:=dynamic enable_depth_publisher:=true`.
  - Verify TF frame exists: `ros2 run tf2_ros tf2_echo world mujoco_camera_wrist_cam`.
  - Verify IMU frame_id matches: `ros2 topic echo --once /mujoco/wrist_cam/imu` shows `frame_id: mujoco_camera_wrist_cam`.
  - Trigger preshaping and confirm the service logs show 2 cameras resolved (front + wrist).
  - **Rationale**: End-to-end verification that the fix works.
  - **Note**: Changes are string-literal-only. Full build requires Docker environment. Verified zero remaining references to old frame name and consistent usage across all 4 locations.

### Phase 2: Documentation Cleanup (P3)

- [x] **Task 2.1**: Remove the "old simulation" section from `README.md`.
  - Remove or replace lines 205-243 of `README.md`. The section documents a `cargo run -- --mode ros` CLI that no longer exists (the crate has no `main.rs`; it is a library-only crate exporting a `cdylib`).
  - Replace with a brief note: "The legacy standalone CLI has been removed. Use the interactive or dynamic simulation described above, which integrates the preshaping service automatically via the launch file."
  - **Rationale**: Following stale instructions leads to build errors and confusion about the current architecture.

- [x] **Task 2.2**: Remove the `grasp_start` topic entry from `README.md`.
  - Remove lines 91-92 documenting `/mujoco/grasp_start`. No code in the codebase publishes or subscribes to this topic.
  - **Rationale**: Documenting nonexistent topics is misleading.

---

## Verification Criteria

1. **Frame consistency**: `ros2 topic echo --once /mujoco/wrist_cam/imu` reports `frame_id: mujoco_camera_wrist_cam`, and `ros2 run tf2_ros tf2_echo world mujoco_camera_wrist_cam` resolves successfully.
2. **Preshaping service**: `ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger` returns `success=true` with 2 cameras resolved in the log output.
3. **Documentation accuracy**: `README.md` contains no references to the removed Rust CLI or the nonexistent `grasp_start` topic.
4. **Build**: `colcon build --packages-select grasp_preshaping mia_hand_mujoco` succeeds cleanly.

## Potential Risks and Mitigations

1. **Downstream consumers expect `mujoco_wrist_cam`**
   Mitigation: If any external node (e.g., on the Jetson multiview system) relies on the old `mujoco_wrist_cam` frame ID, it will break. Search the multiview codebase for this string before merging. The frame name is configurable via the `camera_frames` parameter on the preshaping bridge, so a legacy override is possible.

2. **README removal confuses users who bookmarked old instructions**
   Mitigation: The replacement note explicitly points to the current interactive/dynamic workflow, so the transition path is clear.

## Deferred Items (for future work)

- Covariance bridging (Finding 2) — pass ROS covariance through FFI to Rust predictor
- Planner GUI real twist computation (Finding 3)
- Example verification and warning removal (Finding 6)
- Topic namespacing `/hand_pose` → `/mujoco/hand_pose` (Finding 7)
