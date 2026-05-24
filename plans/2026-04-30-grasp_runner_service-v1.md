# Grasp Runner Service — Integrated Preshaping Trajectory Node

## Objective

Create a new Python ROS 2 node (`grasp_runner_node.py`) that orchestrates a complete grasp approach-preshape-close sequence. The node runs **alongside** the existing `mujoco_interactive` simulation (which provides the MuJoCo GUI, pose topics, depth pipeline, and preshaping bridge). The runner node:

1. Uses the existing `/mujoco/move_hand` smooth motion topic to move the hand toward the object
2. Detects when the hand has sufficient velocity (twist) and calls the preshaping service
3. Subscribes to the preshaping node's output topics (`/grasp_preshaping/wrist_pose`, `/grasp_preshaping/target_finger_closures`, `/grasp_preshaping/target_hand_pose`)
4. After receiving preshaping results: rotates the wrist and partially closes fingers
5. After a 3-second delay: moves the hand to the planned grasp pose, then fully closes the hand
6. Provides a simple "Run Grasp" button via a ROS 2 service trigger (so the existing MuJoCo GUI "Run Planner" button can be repurposed or a separate `ros2 service call` can be used)
7. Swaps test objects by moving them far away (since MuJoCo cannot spawn/despawn at runtime)
8. Uses only the **wrist-mounted camera** (`wrist_cam`) as the primary viewport — camera and hand on the same side to mimic a head-mounted camera on a person

---

## Findings Summary

### Architecture Verified

| Component | Location | Role |
|-----------|----------|------|
| `InteractiveSimulator` | `docker_ws/dev/mujoco/interactive_simulator/interactive_simulator.hpp` | C++ singleton; MuJoCo physics + GUI |
| `InteractiveSystemInterface` | `docker_ws/dev/mujoco/interactive_simulator/interactive_system_interface.hpp` | ros2_control hardware interface; bridges ROS topics ↔ simulator |
| `PreshapingServiceBridgeNode` | `docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | ROS service `/grasp_preshaping/compute_grasp`; calls Rust FFI; publishes results |
| `GraspProximityControllerNode` | `docker_ws/dev/grasp_preshaping/nodes/grasp_proximity_controller_node.py` | Existing proximity-based controller (reference, not used directly) |
| `WristControllerNode` | `docker_ws/dev/mujoco/nodes/wrist_controller_node.py` | Wrist rotation via `/wrist/set_position` (Float64MultiArray `[deg, accel]`) |
| `HandTrajectoryNode` | `docker_ws/dev/mujoco/nodes/mujoco_hand_trajectory_node.py` | Reference: fixed A→B trajectory using `/mujoco/move_hand` |
| `MujocoSceneStatePublisher` | `docker_ws/dev/mujoco/nodes/mujoco_scene_state_publisher_node.py` | Depth/TF publisher (runs inside the same launch) |

### Topics Verified

**Available for the runner node (all already published by InteractiveSystemInterface):**
- `/mujoco/hand_pose` (Pose, ~10 Hz) — current hand position
- `/mujoco/object_pose` (Pose, ~10 Hz) — current object position
- `/hand_twist` (TwistWithCovarianceStamped, ~10 Hz) — hand velocity in hand frame
- `/mujoco/sim_time` (Float64, ~10 Hz) — simulation time

**Control topics (runner publishes to these):**
- `/mujoco/move_hand` (PoseStamped, stamp=duration) — smooth hand motion
- `/mujoco/set_hand_pose` (Pose) — instant teleport
- `/mujoco/move_object` (PoseStamped, stamp=duration) — smooth object motion
- `/mujoco/set_object_pose` (Pose) — instant teleport
- `/thumb_pos_ff_controller/commands` (Float64MultiArray [pos_rad]) — thumb closure
- `/index_pos_ff_controller/commands` (Float64MultiArray [pos_rad]) — index closure
- `/mrl_pos_ff_controller/commands` (Float64MultiArray [pos_rad]) — MRL closure
- `/wrist/set_position` (Float64MultiArray [deg, accel_deg_s2]) — wrist rotation

**Preshaping outputs (runner subscribes to these):**
- `/grasp_preshaping/wrist_pose` (Float64) — wrist rotation in degrees [0, 360)
- `/grasp_preshaping/target_finger_closures` (Float64MultiArray [thumb, index, mrl]) — full closure 0.0–1.0
- `/grasp_preshaping/target_hand_pose` (Pose) — target grasp position in world frame
- `/grasp_preshaping/grasp_type` (Int32) — 1=cylindrical, 2=pinch, 3=lateral

**Preshaping service:**
- `/grasp_preshaping/compute_grasp` (std_srvs/srv/Trigger)

### Scene XML Findings

The wrist camera already exists in the scene (`mia_hand_right_grasp_frame.xml:134`):
```xml
<body name="wrist_cam_body" pos="0 0.095 -0.04">
  <camera name="wrist_cam" pos="0 0 0" euler="-1.5708 0 0" fovy="60"/>
```
It is mounted on the back of the hand (child of `palm_r`), looking toward the fingertips. The user can switch the MuJoCo viewport to `wrist_cam` via the Rendering panel's Camera dropdown.

### Object Swapping Strategy

MuJoCo does not support runtime spawn/despawn. The approach is:
- The current cylinder scene (`scene_right_cylinder.xml`) has `target_cylinder_body` at a fixed position
- The runner node can move the current object far away (e.g., `pos=(0, 0, -10)`) via `/mujoco/set_object_pose`
- For "swapping" objects, we'd need a scene XML with multiple objects placed far away initially, and the runner moves the desired one into the grasp position

### Key Constraint: Closure Value Mapping

The preshaping node publishes closure values in `[0.0, 1.0]`. The finger controllers expect **radians** in the joint range:
- Thumb (`j_thumb_fle_r`): range `[0, 1.1345]`
- Index (`j_index_fle_r`): range `[-1.4, 1.4]`
- MRL (`j_mrl_fle_r`): range `[0, 1.39626]`

The existing `preshaping_service_bridge_node.cpp` already applies `preshaping_closure_fraction` and `min_closure_amount` before publishing to the controllers. However, the runner node needs to do its own mapping from the `[0, 1]` closure to joint-space radians.

Looking at the existing bridge code (`preshaping_service_bridge_node.cpp:403-412`), it publishes the raw closure values (0.0–1.0) to `/grasp_preshaping/target_finger_closures`, and the scaled-down preshape closures directly to the controllers. The runner node will need to map these 0.0–1.0 values to the actual joint position ranges.

---

## Implementation Plan

### Phase 1: Create the Grasp Runner Node

- [ ] **1.1 Create `docker_ws/dev/mujoco/nodes/grasp_runner_node.py`** — A new Python ROS 2 node that implements a state machine for the grasp sequence. States: `IDLE → APPROACHING → CALLING_PRESHAPING → PRESHAPING_RECEIVED → WRIST_AND_PARTIAL_CLOSE → DELAY → MOVING_TO_GRASP → CLOSING → DONE`

- [ ] **1.2 Implement state: IDLE** — Node waits for a trigger. The trigger can be either:
  - A ROS 2 service `/grasp_runner/trigger` (std_srvs/srv/Trigger) for external invocation
  - A ROS 2 topic `/grasp_runner/command` (std_msgs/String) for simple command-line control
  - On trigger: compute an approach waypoint (e.g., 0.15 m offset from the current object position along the hand-to-object vector), move the object to a test position if swapping is needed, then transition to APPROACHING

- [ ] **1.3 Implement state: APPROACHING** — Publish a PoseStamped to `/mujoco/move_hand` with the approach waypoint. Use a duration of ~2 seconds. Start monitoring `/hand_twist` for velocity magnitude. When `||linear_velocity|| > threshold` (e.g., 0.02 m/s), call the preshaping service and transition to CALLING_PRESHAPING

- [ ] **1.4 Implement state: CALLING_PRESHAPING** — Call `/grasp_preshaping/compute_grasp` asynchronously. While waiting, continue moving the hand. When the service responds with success, subscribe to the preshaping output topics and transition to PRESHAPING_RECEIVED. If the service fails, log the error and transition to DONE

- [ ] **1.5 Implement state: PRESHAPING_RECEIVED** — Wait until all three preshaping topics have been received (`wrist_pose`, `target_finger_closures`, `target_hand_pose`). Log the received values (grasp type, scores, closures, wrist angle, target pose). Then immediately:
  - Publish wrist rotation to `/wrist/set_position` with the planned angle
  - Publish partial finger closures (e.g., 30% of the full closure) to the finger controllers
  - Stop the hand motion by publishing current position as a set pose
  - Transition to WRIST_AND_PARTIAL_CLOSE

- [ ] **1.6 Implement state: WRIST_AND_PARTIAL_CLOSE** — Wait for a configurable delay (default 3 seconds). Log progress. Transition to DELAY→MOVING_TO_GRASP

- [ ] **1.7 Implement state: MOVING_TO_GRASP** — Publish a PoseStamped to `/mujoco/move_hand` targeting the `/grasp_preshaping/target_hand_pose`. Use a duration of ~2 seconds. Monitor `/mujoco/hand_pose` vs target; when the distance is below a threshold (e.g., 0.02 m), transition to CLOSING

- [ ] **1.8 Implement state: CLOSING** — Publish the full closure values from `/grasp_preshaping/target_finger_closures` to the finger controllers. Map from [0, 1] to joint radians using the joint limits. Wait ~1 second for the fingers to close. Transition to DONE

- [ ] **1.9 Implement state: DONE** — Log completion status. Stay in this state until a new trigger is received, then reset and go back to IDLE

### Phase 2: Object Swapping Support

- [ ] **2.1 Add test object positions to the node** — Define a set of named test objects with their world positions (e.g., "cylinder_center", "sphere_left"). On trigger, the node moves the current MuJoCo object to the selected test position via `/mujoco/set_object_pose` or `/mujoco/move_object`

- [ ] **2.2 Implement "move far away" for unused objects** — Since the scene only has one object body, swapping means moving the current object to the desired test position. For future multi-object scenes, the node would move unused objects to `(0, 0, -10)` to hide them

### Phase 3: Camera Configuration

- [ ] **3.1 Create a new scene XML** — Create `docker_ws/dev/mujoco/scenes/scene_right_runner.xml` based on `scene_right_cylinder.xml` but with the `depth_cam_body` removed or repositioned. The wrist camera (`wrist_cam_body`, already a child of `palm_r`) serves as the primary camera. This ensures camera and hand are on the same side

- [ ] **3.2 Verify wrist camera works for depth pipeline** — The existing depth publisher uses `front_depth_cam` by default. For the runner scenario, either:
  - Option A: Keep the front camera but position it on the same side as the hand (modify the scene XML)
  - Option B: Switch the depth publisher to use `wrist_cam` by setting `depth_camera_name:=wrist_cam` and `depth_frame_id:=mujoco_camera_wrist_cam` in the launch args
  - Option C: Keep both cameras; the wrist cam is just for viewport viewing, the front cam provides the depth data for the preshaping pipeline
  - **Recommended**: Option C is simplest — keep the existing front depth camera for the preshaping pipeline (it provides the point cloud), but instruct the user to switch the MuJoCo viewport to `wrist_cam` for the head-mounted camera feel

### Phase 4: Docker Integration

- [ ] **4.1 Add a `mujoco_grasp_runner` service to docker-compose** — Similar to the existing `mujoco_trajectory` service. It launches the interactive simulation with the runner scene, then starts the runner node:

  ```yaml
  mujoco_grasp_runner:
    extends: miahand_ros2
    container_name: miahand_ros2_mujoco_grasp_runner
    environment:
      - MUJOCO_OBJECT=${MUJOCO_OBJECT:-cylinder}
    command: >
      bash -c "
        cd /miahand_ws &&
        source /opt/ros/jazzy/setup.bash &&
        colcon build --packages-select mia_hand_mujoco &&
        source install/setup.bash &&
        ros2 launch mia_hand_mujoco mia_hand_system_interface_launch.py scene:=cylinder depth_target_geom_name:=target_cylinder enable_depth_publisher:=true hardware_plugin:=mia_hand_mujoco/InteractiveSystemInterface &
        sleep 8 &&
        ros2 run mia_hand_mujoco grasp_runner_node.py ;
        wait
      "
  ```

- [ ] **4.2 Register the new node in CMakeLists.txt** — Add `grasp_runner_node.py` to the `install(PROGRAMS ...)` section in `docker_ws/mia_hand_mujoco/CMakeLists.txt` alongside the existing `mujoco_hand_trajectory_node.py` and `wrist_controller_node.py`

### Phase 5: Verification and Testing

- [ ] **5.1 Test the node launch** — Verify the node starts, connects to all topics, and logs readiness

- [ ] **5.2 Test the approach phase** — Trigger the runner, verify the hand moves toward the object

- [ ] **5.3 Test preshaping integration** — Verify the service call happens when velocity threshold is met, and that the response topics are received and logged

- [ ] **5.4 Test the full sequence** — Verify wrist rotation → partial close → delay → move to grasp → full close

- [ ] **5.5 Test object swapping** — Verify the node can move the cylinder to different test positions

---

## Verification Criteria

- [ ] The node launches without errors alongside the existing `mujoco_interactive` simulation
- [ ] Triggering the runner causes the hand to move toward the object
- [ ] The preshaping service is called when the hand has velocity
- [ ] Preshaping results (wrist angle, closures, target pose, grasp type) are logged to console
- [ ] After preshaping: wrist rotates and fingers partially close
- [ ] After 3-second delay: hand moves to the planned grasp pose
- [ ] After arriving at the grasp pose: fingers fully close
- [ ] The wrist camera viewport provides a same-side view of the hand and object
- [ ] Object can be repositioned to test different grasp scenarios

---

## Potential Risks and Mitigations

1. **Preshaping service fails due to missing data**
   - The preshaping bridge requires `/hand_pose`, `/hand_twist`, and `/segmented_object_cloud` to all be available before it will compute
   - Mitigation: The runner node should verify these topics are publishing before triggering; log a clear error if the service returns "No pose/twist/cloud data"

2. **Closure value mapping from [0,1] to radians is incorrect**
   - The preshaping node outputs normalized closure values, but the finger controllers expect radians within joint limits
   - Mitigation: Use the joint ranges from the URDF/scene XML (thumb: 0–1.1345, index: -1.4–1.4, MRL: 0–1.39626). Map closure 0.0 → min, 1.0 → max. Verify against the existing bridge node's behavior

3. **Timing: hand may overshoot the approach waypoint before preshaping completes**
   - The approach motion uses smooth interpolation with a fixed duration; if preshaping takes longer than expected, the hand may stop at the approach point and wait
   - Mitigation: Use a sufficiently long approach distance and duration. The runner can also cancel the approach motion and re-publish a stop command

4. **Object body name mismatch between scene variants**
   - The sphere scene uses `target_sphere_body`, the cylinder scene uses `target_cylinder_body`
   - Mitigation: Make the object body name configurable via a ROS parameter, or detect it from the scene

5. **Wrist camera depth not suitable for preshaping**
   - The preshaping pipeline expects a specific camera frame and point cloud format
   - Mitigation: Keep the existing front depth camera for the pipeline; the wrist camera is only for visual viewport. Document this clearly

6. **The existing GUI "Run Planner" button calls the preshaping service directly**
   - If the user presses the GUI button while the runner is active, both will try to call the service simultaneously
   - Mitigation: The runner uses its own trigger mechanism. Document that the GUI "Run Planner" button should not be used when the runner is active (or disable it by not connecting the GUI planner trigger)

---

## Alternative Approaches

1. **Extend the existing `HandTrajectoryNode`** instead of creating a new node
   - Trade-off: The trajectory node is a simple A→B mover with no state machine. Extending it would require significant refactoring. A new node is cleaner and doesn't risk breaking the existing trajectory functionality.

2. **Use the existing `GraspProximityControllerNode`** for the proximity-based closing logic
   - Trade-off: The proximity controller is designed for continuous operation (always running, always checking distance). The runner needs a sequential state machine. The proximity controller could be used as a downstream consumer of the runner's preshaping results, but the runner itself needs its own orchestration logic.

3. **Implement as a C++ node** for tighter integration with the InteractiveSystemInterface
   - Trade-off: Much more complex to implement and build. The Python approach is faster to develop, easier to debug, and has sufficient performance for this orchestration task. All the necessary ROS topics are already exposed.

4. **Modify the InteractiveSimulator C++ code** to add a new GUI section for the runner
   - Trade-off: Would provide a native GUI button but requires C++ changes, recompilation, and is harder to iterate on. The ROS service/topic trigger approach is more flexible.

---

## Detailed Node Design: `grasp_runner_node.py`

### Parameters
- `approach_distance_m` (double, default: 0.15) — How far from the object the approach waypoint is
- `approach_duration_s` (double, default: 2.0) — Duration of the approach motion
- `velocity_threshold_m_s` (double, default: 0.02) — Minimum hand velocity to trigger preshaping
- `partial_closure_factor` (double, default: 0.3) — Fraction of full closure for preshape
- `grasp_delay_s` (double, default: 3.0) — Delay between preshape and grasp approach
- `grasp_approach_duration_s` (double, default: 2.0) — Duration of the move to grasp pose
- `close_duration_s` (double, default: 1.0) — Wait time for fingers to close
- `proximity_threshold_m` (double, default: 0.02) — Distance threshold for "arrived at target"
- `object_position_x` (double, default: -0.1) — Test object X position
- `object_position_y` (double, default: -0.05) — Test object Y position
- `object_position_z` (double, default: 0.31) — Test object Z position

### State Machine
```
IDLE ──[trigger]──→ APPROACHING ──[vel > thresh]──→ CALLING_PRESHAPING
                                                      │
                                        [service success]
                                                      ↓
                                              PRESHAPING_RECEIVED
                                                      │
                                        [all topics received]
                                                      ↓
                                          WRIST_AND_PARTIAL_CLOSE
                                                      │
                                           [3s delay expires]
                                                      ↓
                                            MOVING_TO_GRASP
                                                      │
                                        [distance < threshold]
                                                      ↓
                                               CLOSING ──→ DONE
```

### Topic Subscriptions
- `/mujoco/hand_pose` (Pose) — track current hand position
- `/mujoco/object_pose` (Pose) — track current object position
- `/hand_twist` (TwistWithCovarianceStamped) — detect hand velocity
- `/mujoco/sim_time` (Float64) — simulation time
- `/grasp_preshaping/wrist_pose` (Float64) — planned wrist rotation
- `/grasp_preshaping/target_finger_closures` (Float64MultiArray) — planned closures
- `/grasp_preshaping/target_hand_pose` (Pose) — planned grasp position
- `/grasp_preshaping/grasp_type` (Int32) — planned grasp type

### Publishers
- `/mujoco/move_hand` (PoseStamped) — smooth hand motion
- `/mujoco/set_hand_pose` (Pose) — instant hand teleport
- `/mujoco/move_object` (PoseStamped) — smooth object motion
- `/mujoco/set_object_pose` (Pose) — instant object teleport
- `/thumb_pos_ff_controller/commands` (Float64MultiArray) — thumb position
- `/index_pos_ff_controller/commands` (Float64MultiArray) — index position
- `/mrl_pos_ff_controller/commands` (Float64MultiArray) — MRL position
- `/wrist/set_position` (Float64MultiArray) — wrist rotation

### Services
- `/grasp_runner/trigger` (std_srvs/srv/Trigger) — start the grasp sequence
- `/grasp_runner/reset` (std_srvs/srv/Trigger) — reset to IDLE

### Service Clients
- `/grasp_preshaping/compute_grasp` (std_srvs/srv/Trigger) — call the preshaping planner
