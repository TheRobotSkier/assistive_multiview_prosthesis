# Realistic Approach Behavior Simulation

## Objective

Transform the `mujoco_trajectory` compose service from a simple "teleport to start, then move to end" script into a realistic approach behavior simulation that mimics how a prosthetic hand user would interact with an object. The system should:

1. Move the hand toward the object while the grasp planner runs (The hands needs to move before we start the planner to make sure it has accurate twist information)
2. Immediately apply preshaping (wrist rotation + partial hand closure) once the planner returns
3. Begin progressive hand closure as the hand approaches the planned grasp position

Additionally, reposition the free-floating depth camera (`depth_cam_body`) to the same side of the object as the wrist-mounted camera, since the free-floating camera should emulate a head-mounted camera (which would naturally be on the same side as the user's eyes/head).

---

## Analysis

### Current Architecture

The system has these key components:

- **`hand_trajectory_node.py`** (`docker_ws/dev/mujoco/nodes/mujoco_hand_trajectory_node.py:1-153`): Current trajectory node. Teleports hand to `HAND_START`, waits 1.5s, then sends a 4s smooth motion to `HAND_END`. No integration with the grasp planner.
- **`preshaping_service_bridge_node.cpp`** (`docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:1-382`): ROS service node that calls the Rust planner via FFI. Subscribes to `/hand_pose`, `/hand_twist`, `/segmented_object_cloud`. Publishes finger commands to `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`. Also publishes `/grasp_preshaping/wrist_pose`.
- **`InteractiveSystemInterface`** (`docker_ws/dev/mujoco/interactive_simulator/interactive_system_interface.cpp:1-810`): The hardware plugin that bridges MuJoCo sim with ros2_control. Publishes hand/object/camera poses at ~10Hz, computes twist from finite differences, publishes IMU data, handles smooth motion commands, and triggers the preshaping service when the GUI requests it.
- **`config.rs`** (`docker_ws/dev/grasp_preshaping/src/config.rs:1-32`): All tunable constants. Currently has TSDF, collision, ROI prediction, and covariance constants but no approach-behavior parameters.
- **Scene XMLs** (e.g., `scene_right_dynamic.xml:40`): The `depth_cam_body` is positioned at `pos="-0.08 -0.46 0.36"` which is behind and below the object, looking upward. The wrist camera is mounted on the hand (`mia_hand_right_grasp_frame.xml:126`).

### Data Flow for Approach Behavior

The preshaping service bridge already publishes finger commands. The key gap is that nothing coordinates: (a) calling the planner during motion, (b) applying partial closure during approach, and (c) ramping to full closure near the target.

### Approach Trigger Options

You proposed two approaches. Let me compare them:

**Option A: Cluster-based approach** (your first idea)
- Cluster successful grasp positions across wrist rotations and grasp types
- Begin closing when the hand enters a cluster region
- Pros: More robust to planner uncertainty; accounts for multiple viable grasps
- Cons: Significantly more complex; requires running the planner multiple times or storing candidate grasps; clustering adds a non-trivial algorithmic component; overkill for a simulation

**Option B: Distance threshold from selected grasp position** (your second idea)
- After the planner returns a single best grasp, compute Euclidean distance from current hand position to that grasp position
- When within threshold, begin closing
- Pros: Dead simple; deterministic; easy to tune via config; matches real prosthetic behavior (close when you're "close enough")
- Cons: Sensitive to the quality of the single selected grasp; doesn't account for approach angle

**Recommendation: Option B (distance threshold) is the right choice.** It is simpler, more predictable, easier to debug, and closely mirrors how real myoelectric controllers work -- the user decides when they're close enough and the hand closes. The distance threshold just automates that decision. The cluster approach adds complexity without proportional benefit for a simulation. If you later find the single grasp is unreliable, you can extend to a top-N average, but start simple.

---

## Implementation Plan

### Phase 1: Config Parameters for Approach Behavior

- [ ] **Task 1.1**: Add approach behavior constants to `config.rs` (`docker_ws/dev/grasp_preshaping/src/config.rs`). These are Python-side config for the trajectory node, but since you want them in a central config, define them as ROS parameters on the trajectory node with sensible defaults. Add parameters for:
  - `APPROACH_CLOSURE_START_DISTANCE_M` (e.g., 0.08m) -- distance at which progressive closure begins
  - `APPROACH_FULL_CLOSURE_DISTANCE_M` (e.g., 0.02m) -- distance at which hand is fully closed
  - `PRESHAPING_CLOSURE_PERCENT` (e.g., 0.3) -- how much to close the hand immediately after planner returns (preshape)
  - `PLANNER_WAIT_TIMEOUT_S` (e.g., 5.0) -- max time to wait for planner result

  Rationale: These are the tuning knobs that control the approach behavior. Having them as ROS parameters allows runtime adjustment without rebuilding.

### Phase 2: Refactor hand_trajectory_node.py into ApproachBehaviorNode

- [ ] **Task 2.1**: Rename/refactor `mujoco_hand_trajectory_node.py` (`docker_ws/dev/mujoco/nodes/mujoco_hand_trajectory_node.py:1-153`) into a new `mujoco_approach_behavior_node.py`. The new node should be a long-running ROS node (not a fire-and-forget script) that subscribes to topics and runs a state machine.

  Rationale: The current node is a script that sends one trajectory and exits. The approach behavior needs to be reactive -- it must monitor hand position, wait for planner results, and progressively close the hand.

- [ ] **Task 2.2**: Implement a state machine with these states:
  1. **INIT**: Teleport hand to start position, wait for sim readiness (reuse existing logic)
  2. **MOVING_TO_OBJECT**: Begin smooth hand motion toward the object. Call the grasp planner service (`/grasp_preshaping/compute_grasp`) at the start of motion. Wait for planner result.
  3. **PRESHAPING**: Once planner returns, immediately apply wrist rotation (from `/grasp_preshaping/wrist_pose`) and partial hand closure (`PRESHAPING_CLOSURE_PERCENT`). Transition to APPROACHING.
  4. **APPROACHING**: Continue monitoring hand position vs. planner grasp position. Compute distance. When within `APPROACH_CLOSURE_START_DISTANCE_M`, begin linearly interpolating hand closure from `PRESHAPING_CLOSURE_PERCENT` to 1.0 as distance goes from start threshold to `APPROACH_FULL_CLOSURE_DISTANCE_M`.
  5. **GRASPED**: Hand is fully closed. Log success and hold position.

  Rationale: The state machine cleanly separates the phases of approach behavior and makes the logic debuggable. Each state has clear entry/exit conditions.

- [ ] **Task 2.3**: Add subscriptions to:
  - `/mujoco/hand_pose` (geometry_msgs/Pose) -- track current hand position
  - `/mujoco/sim_time` (std_msgs/Float64) -- for velocity computation
  - `/grasp_preshaping/wrist_pose` (geometry_msgs/Pose) -- to get the planned wrist orientation
  - `/mujoco/object_pose` (geometry_msgs/Pose) -- to know where the object is (for initial motion target)

  Rationale: These subscriptions provide the real-time data needed for the distance-based closure logic.

- [ ] **Task 2.4**: Add a service client to `/grasp_preshaping/compute_grasp` (std_srvs/Trigger). Call it once at the start of the MOVING_TO_OBJECT state. Store the result (grasp type, closure amounts, wrist orientation).

  Rationale: The planner needs to run while the hand is in motion to get a realistic result. The current architecture already handles this -- the service bridge reads the latest pose/twist/cloud and calls the Rust backend.

- [ ] **Task 2.5**: Implement the distance-based progressive closure logic. In the APPROACHING state, on each tick:
  1. Compute Euclidean distance between current hand position and the planner's grasp position (from the grasp pose embedded in the planner result -- note: the planner returns wrist quaternion but the grasp position is the hand position at the time of planning, which is stored in the service response message or can be derived from the planned grasp target)
  2. Map distance to closure amount: `closure = lerp(PRESHAPING_CLOSURE_PERCENT, 1.0, 1.0 - clamp((dist - FULL_DIST) / (START_DIST - FULL_DIST), 0, 1))`
  3. Publish finger commands to `/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`

  Rationale: This is the core of the approach behavior -- the hand progressively closes as it nears the target, giving the user visual feedback.

  **Important design note**: The current planner service (`std_srvs/Trigger`) does not return the grasp position -- it only returns success/message. The finger commands are published internally by the bridge node. To implement distance-based closure, we need either:
  - (a) Extend the service to return the grasp position, OR
  - (b) Use the initial hand position as an approximation of the grasp target (since the planner optimizes for where the hand will be), OR
  - (c) Subscribe to the finger closure commands published by the bridge and use the object position as the target

  **Recommended**: Option (b) is simplest -- record the hand position when the planner is called, and use that as the "grasp target position" for distance computation. The planner optimizes grasp poses near the current hand trajectory, so the planned grasp position is close to where the hand will be. Alternatively, use the object position as the target since that's what the hand is approaching.

  **Simplest viable approach**: Use the object position (from `/mujoco/object_pose`) as the grasp target. The hand is moving toward the object, so distance to object = distance to grasp. This avoids any service changes.

- [ ] **Task 2.6**: Apply wrist rotation from the planner result. Subscribe to `/grasp_preshaping/wrist_pose` and use `/mujoco/move_hand` to update the hand orientation to match the planned wrist orientation while the hand is in motion.

  Rationale: The planner outputs an optimal wrist orientation. In the real system, the wrist would rotate to match. In sim, we apply this via the smooth motion topic.

### Phase 3: Integrate with Docker Compose

- [ ] **Task 3.1**: Update the `mujoco_trajectory` service in `docker-compose.yml` (`docker_ws/docker-deployment/docker-compose.yml:109-125`) to launch the new approach behavior node instead of the old trajectory node. Change the `ros2 run` command from `mujoco_hand_trajectory_node.py` to the new `mujoco_approach_behavior_node.py`.

  Rationale: The compose service already launches the interactive sim + trajectory node. We just swap the trajectory node for the approach behavior node.

- [ ] **Task 3.2**: Ensure the approach behavior node is registered as an entry point in the `mia_hand_mujoco` package's `setup.py` or `CMakeLists.txt` so it can be launched via `ros2 run`.

  Rationale: The node needs to be discoverable by ROS 2.

### Phase 4: Camera Repositioning

- [ ] **Task 4.1**: In all scene XML files (`scene_right_dynamic.xml`, `scene_right_static.xml`, `scene_right_custom.xml`, `scene_right_cylinder.xml`), move the `depth_cam_body` position to the same side of the object as the wrist camera. Currently the depth camera is at `pos="-0.08 -0.46 0.36"` (behind and below). The wrist camera is on the hand which approaches from above/in front.

  The new position should place the free-floating camera roughly where a user's head would be -- above and behind the hand, looking down toward the object. A good starting position would be something like `pos="-0.08 0.15 0.55"` with an appropriate euler angle to look down at the object (approximately `euler="-2.2 0 0"` to tilt the camera down). The exact values will need tuning based on the scene geometry.

  Rationale: The free-floating camera emulates a head-mounted camera. A user's head is above and behind their hand, on the same side as the hand, not on the opposite side of the object. This also helps the TSDF sign calculation since both cameras will see the same side of the object.

- [ ] **Task 4.2**: Verify that the camera euler angles produce a valid depth image of the object. The camera's `euler` attribute in MuJoCo uses the same convention as the scene. The `front_depth_cam` child camera has `euler="3.14 0 0"` which flips it 180 degrees (MuJoCo camera convention). The parent body euler needs to orient the camera toward the object.

  Rationale: Just moving the position without adjusting orientation will result in the camera pointing in the wrong direction.

---

## Verification Criteria

- [ ] The approach behavior node teleports the hand to start, begins moving toward the object, and calls the grasp planner during motion
- [ ] Upon planner result, the hand immediately applies partial closure (preshape) and wrist rotation
- [ ] As the hand approaches the object, finger closure increases linearly from preshape percentage to full closure
- [ ] The distance threshold values are configurable via ROS parameters
- [ ] The free-floating camera (`depth_cam_body`) is positioned on the same side as the wrist camera (user's side), producing a valid point cloud of the object
- [ ] The `mujoco_trajectory` compose service launches and completes the full approach behavior sequence

## Potential Risks and Mitigations

1. **Planner service returns before motion starts or after hand passes the object**
   Mitigation: The state machine should call the planner at the start of motion. The planner uses the current pose/twist, so calling it early in the trajectory gives it realistic input. Add a timeout and fallback (e.g., close fully at a fixed distance from object).

2. **Distance to object is not a good proxy for distance to planned grasp position**
   Mitigation: Start with object distance as the proxy. If this proves inaccurate, extend the service response to include the planned grasp position. This is a minor change to `preshaping_service_bridge_node.cpp` -- add the grasp position to the service response or publish it on a separate topic.

3. **Wrist rotation during motion causes the hand to deviate from the trajectory**
   Mitigation: Apply wrist rotation as a separate orientation update while maintaining the position trajectory. The `/mujoco/move_hand` topic handles position and orientation together, so either use position-only teleport for orientation updates or combine position + orientation in the motion command.

4. **Camera repositioning breaks the point cloud segmentation**
   Mitigation: The `filter_points_by_target_geom` method in `mujoco_scene_state_publisher_node.py:422-463` uses ray casting against the target geom, so it should work regardless of camera position as long as the camera can see the object. Verify by checking the published point cloud after repositioning.

5. **Finger command timing -- the preshaping bridge node and the approach node both publish to the same finger command topics**
   Mitigation: The approach behavior node should take over finger command publishing after the planner returns. The bridge node publishes once when the service is called; the approach node then continuously publishes progressive closure. This is fine because ROS topics use last-wins semantics. Alternatively, disable the bridge node's finger publishing and have the approach node handle all closure logic.

## Alternative Approaches

1. **Extend the preshaping service to return grasp position**: Instead of using object position as the distance target, modify `preshaping_service_bridge_node.cpp` to publish the planned grasp position on a new topic (e.g., `/grasp_preshaping/grasp_target_pose`). The approach node subscribes to this for more accurate distance computation. Trade-off: requires modifying the C++ bridge node and potentially the FFI interface, but gives more precise closure triggering.

2. **Use the InteractiveSystemInterface's planner trigger instead of calling the service separately**: The interactive sim already has planner trigger logic via the GUI. The approach node could reuse this by programmatically triggering the GUI button. Trade-off: tighter coupling to the interactive sim; calling the service directly is cleaner.

3. **Cluster-based approach (your original idea)**: Run the planner multiple times during approach and cluster the results. Begin closing when the hand enters any cluster centroid's neighborhood. Trade-off: much more complex, requires multiple planner calls (each takes time), and the planner's stochastic sampling means results vary between calls. Only worth it if the single-grasp approach proves unreliable.

4. **Time-based closure instead of distance-based**: Instead of computing distance, start closing at a fixed time after motion begins (e.g., start closing at 60% of trajectory duration). Trade-off: simpler but doesn't adapt to varying approach speeds or distances. Distance-based is more realistic.
