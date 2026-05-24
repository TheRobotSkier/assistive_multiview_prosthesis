# Realistic Approach Behavior Simulation

## Objective

Transform the `mujoco_trajectory` compose service from a simple "teleport to start, then move to end" script into a realistic approach behavior simulation that mimics how a prosthetic hand user would interact with an object. The system should:

1. Move the hand toward the object while the grasp planner runs (the hand needs to move **before** the planner is called to ensure accurate twist information)
2. Immediately apply preshaping (wrist rotation + partial hand closure) once the planner returns
3. Begin progressive hand closure as the hand approaches the planned grasp position

Additionally, reposition the free-floating depth camera (`depth_cam_body`) to the same side of the object as the wrist-mounted camera, since the free-floating camera should emulate a head-mounted camera (which would naturally be on the same side as the user's eyes/head).

---

## Analysis

### Approach Trigger: Distance Threshold (Option B)

**Recommended: distance threshold from object position**, defined in `config.rs` as the single source of truth. This is simpler than clustering, deterministic, easy to tune, and closely mirrors how real myoelectric controllers work.

### How Python Reads `config.rs` Values

The Rust library (`libgrasp_preshaping.so`) is already a `cdylib` (`Cargo.toml:7`) loaded by the C++ bridge node via `dlopen`. The Python approach node can use `ctypes` to load the same `.so` and call a new FFI getter function. This keeps `config.rs` as the single source of truth -- no duplicate values in Python.

### Data Flow

1. Hand starts moving toward object via `/mujoco/move_hand`
2. After a short delay (ensuring the hand has velocity and the twist topic has data), the planner service is called
3. The planner reads current pose/twist/cloud and returns grasp type, closure amounts, and wrist quaternion
4. The approach node immediately applies wrist rotation and partial closure (preshape)
5. The approach node monitors distance to object and ramps closure from preshape% to 100%

### Current Architecture References

- `hand_trajectory_node.py` (`docker_ws/dev/mujoco/nodes/mujoco_hand_trajectory_node.py:1-153`): Fire-and-forget script, no planner integration
- `preshaping_service_bridge_node.cpp` (`docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:83-92`): Publishes finger commands and wrist pose
- `InteractiveSystemInterface` (`docker_ws/dev/mujoco/interactive_simulator/interactive_system_interface.cpp:328-574`): Publishes hand/object/camera poses at ~10Hz, twist at ~10Hz
- `config.rs` (`docker_ws/dev/grasp_preshaping/src/config.rs:1-32`): All tunable constants
- `c_api.rs` (`docker_ws/dev/grasp_preshaping/src/c_api.rs:482-486`): Exported FFI functions
- Scene XMLs: `depth_cam_body` at `pos="-0.08 -0.46 0.36"` (opposite side from hand)

---

## Implementation Plan

### Phase 1: Config Constants in `config.rs`

- [ ] **Task 1.1**: Add approach behavior constants to `config.rs` (`docker_ws/dev/grasp_preshaping/src/config.rs`). Add a new section:

  ```
  // Approach behavior (used by the trajectory simulation node)
  pub const APPROACH_CLOSURE_START_DISTANCE_M: f64 = 0.08;
  pub const APPROACH_FULL_CLOSURE_DISTANCE_M: f64 = 0.02;
  pub const PRESHAPING_CLOSURE_PERCENT: f64 = 0.3;
  pub const PLANNER_CALL_DELAY_S: f64 = 0.5;
  ```

  - `APPROACH_CLOSURE_START_DISTANCE_M`: Distance from object at which progressive closure begins
  - `APPROACH_FULL_CLOSURE_DISTANCE_M`: Distance from object at which hand is fully closed
  - `PRESHAPING_CLOSURE_PERCENT`: How much to close immediately after planner returns (preshape)
  - `PLANNER_CALL_DELAY_S`: Time to wait after motion starts before calling the planner (ensures twist data is populated)

  Rationale: These are the core tuning knobs. Defined here so the Rust optimizer can use them and the Python node can read them via FFI.

### Phase 2: FFI Config Getter

- [ ] **Task 2.1**: Add a new `#[repr(C)]` struct and exported FFI function to `c_api.rs` (`docker_ws/dev/grasp_preshaping/src/c_api.rs`) that exposes the approach behavior config values:

  ```
  #[repr(C)]
  pub struct ApproachConfigFFI {
      pub closure_start_distance_m: f64,
      pub full_closure_distance_m: f64,
      pub preshaping_closure_percent: f64,
      pub planner_call_delay_s: f64,
  }

  #[unsafe(no_mangle)]
  pub extern "C" fn grasp_preshaping_approach_config() -> ApproachConfigFFI {
      ApproachConfigFFI {
          closure_start_distance_m: config::APPROACH_CLOSURE_START_DISTANCE_M,
          full_closure_distance_m: config::APPROACH_FULL_CLOSURE_DISTANCE_M,
          preshaping_closure_percent: config::PRESHAPING_CLOSURE_PERCENT,
          planner_call_delay_s: config::PLANNER_CALL_DELAY_S,
      }
  }
  ```

  Rationale: The Python node loads `libgrasp_preshaping.so` via `ctypes` and calls this function to get the config values. This guarantees the Python node and the Rust planner read the same constants. No duplicate values anywhere.

- [ ] **Task 2.2**: Rebuild the Rust library (`colcon build --packages-select grasp_preshaping`). The `.so` is output to `target/release/libgrasp_preshaping.so` and installed to the colcon install space.

### Phase 3: Approach Behavior Node

- [ ] **Task 3.1**: Create `docker_ws/dev/mujoco/nodes/mujoco_approach_behavior_node.py` as a new long-running ROS 2 node (replacing the old trajectory node). The node should use `ctypes` to load `libgrasp_preshaping.so` and call `grasp_preshaping_approach_config()` to read all tuning parameters from the Rust library.

  Rationale: Loading config from the shared library guarantees single source of truth. The Python node never hardcodes distance thresholds.

- [ ] **Task 3.2**: Implement a state machine with these states:

  1. **INIT**: Teleport hand to `HAND_START` (reuse existing logic from `mujoco_hand_trajectory_node.py:104-110`). Wait for sim readiness via `/mujoco/hand_pose`.
  2. **MOVING_TO_OBJECT**: Begin smooth hand motion toward the object via `/mujoco/move_hand`. Wait `PLANNER_CALL_DELAY_S` seconds (from config) to ensure the hand has velocity and the twist topic has meaningful data. Then call `/grasp_preshaping/compute_grasp`.
  3. **PRESHAPING**: Once the planner service returns successfully, immediately:
     - Apply wrist rotation by publishing the wrist quaternion from `/grasp_preshaping/wrist_pose` as an orientation update via `/mujoco/move_hand` (or `/mujoco/set_hand_pose` for instant orientation change)
     - Apply partial hand closure at `PRESHAPING_CLOSURE_PERCENT` by publishing to the finger command topics (`/thumb_pos_ff_controller/commands`, `/index_pos_ff_controller/commands`, `/mrl_pos_ff_controller/commands`)
     - Transition to APPROACHING
  4. **APPROACHING**: On each tick (driven by `/mujoco/hand_pose` subscription at ~10Hz):
     - Compute Euclidean distance between current hand position and object position (from `/mujoco/object_pose`)
     - If distance <= `APPROACH_CLOSURE_START_DISTANCE_M` and > `APPROACH_FULL_CLOSURE_DISTANCE_M`:
       - Compute closure: `closure = lerp(PRESHAPING_CLOSURE_PERCENT, 1.0, 1.0 - clamp((dist - FULL_DIST) / (START_DIST - FULL_DIST), 0, 1))`
       - Publish finger commands with the computed closure value
     - If distance <= `APPROACH_FULL_CLOSURE_DISTANCE_M`:
       - Publish full closure (1.0) and transition to GRASPED
  5. **GRASPED**: Hand is fully closed. Log success and hold position.

  Rationale: The state machine cleanly separates phases. Motion starts before the planner call (per your requirement) so the planner gets accurate twist data. The planner runs while the hand is in motion, just like in a real prosthetic system.

- [ ] **Task 3.3**: Add subscriptions to:
  - `/mujoco/hand_pose` (geometry_msgs/Pose) -- track current hand position for distance computation
  - `/mujoco/object_pose` (geometry_msgs/Pose) -- object position as the distance target
  - `/mujoco/sim_time` (std_msgs/Float64) -- for timing the planner call delay
  - `/grasp_preshaping/wrist_pose` (geometry_msgs/Pose) -- planned wrist orientation

  Rationale: These provide the real-time data needed for distance-based closure. Using object position as the target avoids needing to modify the planner service response.

- [ ] **Task 3.4**: Add a service client to `/grasp_preshaping/compute_grasp` (std_srvs/Trigger). Call it once, `PLANNER_CALL_DELAY_S` seconds after motion begins. Store the result.

  Rationale: The delay ensures the hand has nonzero velocity when the planner reads the twist, giving the predictor meaningful trajectory data to sample from.

- [ ] **Task 3.5**: Implement the distance-based progressive closure logic. Map distance to closure amount linearly between `APPROACH_CLOSURE_START_DISTANCE_M` and `APPROACH_FULL_CLOSURE_DISTANCE_M`. Publish per-finger commands respecting the grasp type (cylindrical: all fingers equal; pinch: thumb+index only, mrl=0; lateral: thumb+index only, mrl=0).

  Rationale: The planner already returns per-finger closure and grasp type. The approach node scales these proportionally as the hand nears the object.

- [ ] **Task 3.6**: Handle the case where the planner service fails or times out. Fallback: use a fixed time-based closure (start closing at 60% of trajectory duration, fully closed at 90%).

  Rationale: The simulation should degrade gracefully, not hang.

### Phase 4: Docker Compose Integration

- [ ] **Task 4.1**: Update the `mujoco_trajectory` service in `docker-compose.yml` (`docker_ws/docker-deployment/docker-compose.yml:109-125`). Change the `ros2 run` command from `mujoco_hand_trajectory_node.py` to the new `mujoco_approach_behavior_node.py`.

  Rationale: The compose service already launches the interactive sim + trajectory node. Just swap the node name.

- [ ] **Task 4.2**: Register the new node as an entry point in the `mia_hand_mujoco` package so it can be launched via `ros2 run mia_hand_mujoco mujoco_approach_behavior_node.py`.

  Rationale: The node must be discoverable by ROS 2 CLI.

### Phase 5: Camera Repositioning

- [ ] **Task 5.1**: In all scene XML files, move `depth_cam_body` to the same side as the user (where the hand approaches from). Current position `pos="-0.08 -0.46 0.36"` (behind and below, opposite side from hand). New position should be above and behind the hand's approach path, looking down at the object. A reasonable starting point: `pos="-0.1 0.25 0.55"` with `euler="-2.5 0 0"` (tilted down toward the object). This places the camera where a user's head would be.

  Files to update:
  - `docker_ws/dev/mujoco/scenes/scene_right_dynamic.xml:40`
  - `docker_ws/dev/mujoco/scenes/scene_right_static.xml:43`
  - `docker_ws/dev/mujoco/scenes/scene_right_custom.xml:42`
  - `docker_ws/dev/mujoco/scenes/scene_right_cylinder.xml:41`

  Rationale: The free-floating camera emulates a head-mounted camera. A user's head is on the same side as their hand, not on the opposite side of the object. This also improves TSDF sign calculation since both cameras see the same hemisphere of the object.

- [ ] **Task 5.2**: Verify the camera euler angles produce a valid depth image of the object. The `front_depth_cam` child has `euler="3.14 0 0"` (180-degree flip for MuJoCo camera convention). The parent body euler must orient the camera to look down at the object from the new position. Adjust as needed.

  Rationale: Position without correct orientation = camera pointing at the ceiling.

---

## Verification Criteria

- [ ] The approach behavior node teleports the hand to start, begins moving toward the object, and calls the grasp planner **after** a configurable delay (ensuring nonzero twist)
- [ ] Upon planner result, the hand immediately applies partial closure (preshape) and wrist rotation
- [ ] As the hand approaches the object, finger closure increases linearly from preshape percentage to full closure
- [ ] All distance threshold values come from `config.rs` via the FFI getter -- no hardcoded values in Python
- [ ] The free-floating camera is positioned on the same side as the wrist camera, producing a valid point cloud
- [ ] The `mujoco_trajectory` compose service launches and completes the full approach behavior sequence

## Potential Risks and Mitigations

1. **Planner service returns after the hand has already passed the object**
   Mitigation: The `PLANNER_CALL_DELAY_S` is short (0.5s) and the trajectory is 4s, so the planner should return well before the hand reaches the object. Add a timeout (5s) and a fallback to time-based closure.

2. **Dual publishers on finger command topics (bridge node + approach node)**
   Mitigation: The bridge node publishes once when the service is called. The approach node then takes over with continuous progressive closure. ROS last-wins semantics handle this. If confusing, add a parameter to the bridge node to disable its finger publishing when the approach node is active.

3. **Camera repositioning breaks point cloud segmentation**
   Mitigation: The ray-cast filtering in `mujoco_scene_state_publisher_node.py:422-463` works regardless of camera position as long as the camera sees the object. Verify by checking `/segmented_object_cloud` after repositioning.

4. **`ctypes` cannot find `libgrasp_preshaping.so`**
   Mitigation: Use the same search paths as the C++ bridge node (`preshaping_service_bridge_node.cpp:121-131`): check `GRASP_PRESHAPING_LIB_PATH` env var first, then standard install paths. Log a warning and fall back to hardcoded defaults if the library cannot be loaded.

5. **Wrist rotation during motion causes trajectory deviation**
   Mitigation: Apply wrist rotation as an orientation-only update while the position trajectory continues. Use `/mujoco/set_hand_pose` for instant orientation change (position unchanged) or encode orientation into the ongoing motion command.

## Alternative Approaches

1. **Publish config values on a ROS topic instead of FFI**: Have the bridge node read `config.rs` and publish the approach constants on a topic. Simpler than `ctypes` but adds a dependency on the bridge node running. FFI approach is self-contained.

2. **Extend planner service response to include grasp position**: Instead of using object position as the distance target, modify `preshaping_service_bridge_node.cpp` to publish the planned grasp position on `/grasp_preshaping/grasp_target_pose`. More accurate but requires C++ changes. Start with object position; extend if needed.

3. **Cluster-based approach**: Run the planner multiple times during approach and cluster results. Begin closing when entering any cluster region. Much more complex; only worth it if single-grasp proves unreliable.

4. **Time-based closure**: Start closing at a fixed percentage of trajectory duration. Simpler but doesn't adapt to varying approach speeds. Distance-based is more realistic and only marginally more complex.
