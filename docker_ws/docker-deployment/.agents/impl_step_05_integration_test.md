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
- [ ] The simulated Mia Hand wrist rotates and fingers partially close (initial preshape) immediately after plan commit by the proximity controller.
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
# No camera pose env vars needed — the camera node publishes world -> camera TF dynamically.
docker compose --profile standalone up digital_twin
```

### 5.3 Visual Verification

1. **RViz loads** with `digital_twin.rviz`.
2. **FusedCloud** display shows the real pointcloud.
3. **RobotModel** shows the Mia Hand in its default pose.
4. Verify `ros2 run tf2_ros tf2_echo world cam1_d435_1_color_optical_frame` shows the camera TF (published by the camera node).

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
- Simulated hand fingers partially close (initial preshape) **immediately** — this is sent by the proximity controller on plan commit, not the bridge.
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
| No pointcloud in RViz | `multiview_full` not running; camera TF not published | Check `ros2 topic hz /fused_pointcloud`; verify `ros2 run tf2_ros tf2_echo world cam1_d435_1_color_optical_frame` |
| Segmentation node warns "No cloud received yet" | Relay not working | Check `ros2 topic hz /segmentation/input_cloud` |
| Segmentation inference times out | `segmentation_inference` not running | Start inference server; check `curl http://127.0.0.1:5678/health` (or the configured `INFERENCE_URL`) |
| Grasp service returns `No pose data` | `/hand_pose` not published | Verify `InteractiveSystemInterface` is loaded; check `ros2 topic hz /hand_pose` |
| Hand joints do not move after service call | Controllers not active; or preshaping bridge still sending commands | Check `ros2 control list_controllers`; verify `publish_initial_commands:=false` in launch; check proximity controller logs |
| RViz RobotModel is red | `/robot_description` missing | Check `ros2 topic echo /robot_description --once` |

---

## Documentation

Add a brief section to `docker-deployment/README.md` (or as a large comment block in the compose file) covering:

1. **What the digital twin service does** (2 sentences).
2. **Prerequisites** (3 services that must be running first).
3. **Startup command** (`docker compose --profile standalone up digital_twin`).
4. **Interaction steps** (click in RViz → call service → watch hand preshape/close).
5. **Optional trajectory test** (`USE_TRAJECTORY=true`).
6. **Inference URL note** — if inference runs in a separate container, set `INFERENCE_URL` to the host-accessible address.

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
  - **Mitigation:** Ensure the CharUco/AprilTag marker is placed at the same location as the MuJoCo `world` origin (or adjust the MuJoCo scene XML to match). The static scene hand start pose may also need adjustment.
- **Risk:** DDS discovery issues between Humble (multiview) and Jazzy (digital twin) containers.
  - **Mitigation:** Both already use `rmw_cyclonedds_cpp` and `ROS_AUTOMATIC_DISCOVERY_RANGE: LOCALHOST`, which is confirmed working in the project.
- **Risk:** The preshaping bridge and proximity controller both publish to `/*/pos_ff_controller/commands`.
  - **Mitigation:** The bridge is parameterised with `publish_initial_commands:=false` in the digital twin launch. The proximity controller owns all controller command publishing. The bridge still publishes planner topics (`/grasp_preshaping/*`) consumed by the controller.
