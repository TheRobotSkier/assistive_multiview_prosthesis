# Implementation Plan: Step 5 — Integration, Test & Documentation

**Beads Issue:** `integration-test`  
**Type:** task  
**Priority:** 2 (blocked by `docker-service`)  
**Estimated Effort:** Medium

---

## Objective

Validate the complete digital twin pipeline end-to-end, fix any integration issues, and document the startup procedure so team members can run it independently.

---

## Acceptance Criteria

- [ ] The `digital_twin` container starts successfully and RViz opens.
- [ ] `/fused_pointcloud` from `multiview_full` is visible in RViz.
- [ ] Clicking a point in RViz (PublishPoint tool) triggers segmentation and `/segmentation/object_cloud` appears.
- [ ] Calling `/grasp_preshaping/compute_grasp` returns `success=true` and publishes planner topics.
- [ ] The simulated Mia Hand wrist rotates and fingers partially close (preshape) after the service call.
- [ ] When the simulated hand is moved closer to the object (via `/mujoco/move_hand` or trajectory node), fingers fully close within the proximity threshold.
- [ ] A runbook (inline in compose comments or a brief section in README) documents the exact startup order and commands.
- [ ] All beads issues for prior steps are closed.

---

## Test Procedure

### 5.1 Pre-flight Checks

Ensure these are running **before** starting `digital_twin`:

```bash
# 1. Real cameras (in a separate terminal or as background services)
docker compose --profile full_system up multiview_full

# 2. Segmentation inference server
docker compose --profile segmentation up segmentation_inference

# 3. Rust library (one-shot, only if .so is missing)
docker compose --profile standalone up rust_build
```

### 5.2 Start Digital Twin

```bash
# Set camera pose (example: camera is 0.5 m above table, looking down)
export CAMERA_FRAME_WORLD_X=0.0
export CAMERA_FRAME_WORLD_Y=0.0
export CAMERA_FRAME_WORLD_Z=0.5
export CAMERA_FRAME_WORLD_QX=0.0
export CAMERA_FRAME_WORLD_QY=0.0
export CAMERA_FRAME_WORLD_QZ=0.0
export CAMERA_FRAME_WORLD_QW=1.0

docker compose --profile standalone up digital_twin
```

### 5.3 Visual Verification

1. **RViz loads** with `digital_twin.rviz`.
2. **FusedCloud** display shows the real pointcloud (may need to set Fixed Frame to `world` or `cam1_d435_1_color_optical_frame` temporarily if TF is misaligned).
3. **RobotModel** shows the Mia Hand in its default pose.

### 5.4 Segmentation Test

1. In RViz, select the **Publish Point** tool.
2. Click on an object in the pointcloud.
3. Verify in terminal logs:
   - Click relay prints: `[+] positive click at (x, y, z)`
   - Segmentation node prints: `Published segmented cloud: N/M points`
4. In RViz, **SegmentedCloud** display shows orange points around the clicked object.

### 5.5 Grasp Computation Test

In a new terminal (inside the `digital_twin` container or on host with matching DDS domain):

```bash
ros2 service call /grasp_preshaping/compute_grasp std_srvs/srv/Trigger
```

Expected output:
- Service response: `success: true` with a message describing grasp type and closures.
- Preshaping bridge logs the planner result.
- Proximity controller logs: `New plan committed — closures=[...], wrist=...°`
- Simulated hand fingers partially close (preshape).
- Simulated wrist rotates to planned angle.

### 5.6 Proximity Closure Test

**Option A — Manual approach:**
Publish a pose that moves the hand closer to the object:
```bash
ros2 topic pub /mujoco/move_hand geometry_msgs/PoseStamped "{
  header: {stamp: {sec: 4, nanosec: 0}},
  pose: {position: {x: -0.12, y: 0.1, z: 0.31}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
}" --once
```
*(Adjust target pose to match your scene.)*

**Option B — Trajectory node:**
Set `USE_TRAJECTORY=true` before starting the service, or run the trajectory node manually:
```bash
ros2 run mia_hand_mujoco mujoco_hand_trajectory_node.py
```

**Expected behavior:**
- As hand moves, proximity controller compares current pose to planned target pose.
- When distance < `proximity_enter_threshold_m` (default 0.08 m), it logs: `Entered near zone ...`
- Fingers move to **full closure** (planned values, not partial).

### 5.7 Failure Modes to Check

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| No pointcloud in RViz | `multiview_full` not running; TF misaligned | Check `ros2 topic hz /fused_pointcloud`; verify static TF |
| Segmentation node warns "No cloud received yet" | Relay not working | Check `ros2 topic hz /segmentation/input_cloud` |
| Segmentation inference times out | `segmentation_inference` not running | Start inference server; check `curl http://127.0.0.1:5678/health` |
| Grasp service returns `No pose data` | `/hand_pose` not published | Verify `InteractiveSystemInterface` is loaded; check `ros2 topic hz /hand_pose` |
| Hand joints do not move | Controllers not active | Check `ros2 control list_controllers`; ensure `joint_state_broadcaster` and `*_pos_ff_controller` are active |
| RViz RobotModel is red | `/robot_description` missing | Check `ros2 topic echo /robot_description --once` |

---

## Documentation

Add a brief section to `docker-deployment/README.md` (or as a large comment block in the compose file) covering:

1. **What the digital twin service does** (2 sentences).
2. **Prerequisites** (3 services that must be running first).
3. **Environment variables** for camera pose.
4. **Startup command** (`docker compose --profile standalone up digital_twin`).
5. **Interaction steps** (click in RViz → call service → watch hand).
6. **Optional trajectory test** (`USE_TRAJECTORY=true`).

---

## Sub-tasks (beads tracking)

1. Run the full startup sequence and note any issues.
2. Fix topic remapping or TF issues.
3. Fix controller or pose publisher issues.
4. Verify segmentation + grasp + proximity closure chain.
5. Write runbook / README section.
6. Close all beads issues (`rviz-config`, `launch-digital-twin`, `bridge-nodes`, `docker-service`, `integration-test`).
7. Commit and push.

---

## Notes / Risks

- **Risk:** The physical camera setup may not match the default `world` origin assumed by the MuJoCo static scene.
  - **Mitigation:** Document that users must calibrate and set `CAMERA_FRAME_WORLD_*` env vars. The static scene hand start pose may also need adjustment.
- **Risk:** DDS discovery issues between Humble (multiview) and Jazzy (digital twin) containers.
  - **Mitigation:** Both already use `rmw_cyclonedds_cpp` and `ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST`, which is confirmed working in the project.
- **Risk:** The preshaping bridge and proximity controller both publish to `/*/pos_ff_controller/commands`, causing potential command fighting.
  - **Mitigation:** For the first integration test, observe if this is actually a problem. The bridge sends a one-shot preshape; the proximity controller runs at 10 Hz. If fighting occurs, add a parameter to the bridge to disable immediate publishing.
