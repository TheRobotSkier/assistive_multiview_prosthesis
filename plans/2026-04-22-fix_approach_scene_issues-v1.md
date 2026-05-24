# Fix Approach Scene Issues

## Objective

Fix four issues with the `scene:=approach` launch configuration:
1. Head camera pointing the wrong way (needs 180-degree Z rotation)
2. No interactive GUI available for the approach scene
3. Hand starts too close to the ball (by design but needs verification)
4. `approach_controller` reports "No hand pose received yet" because the headless `SystemInterface` doesn't publish pose topics

## Root Cause Analysis

### Issue 1: Camera orientation
The `depth_cam_body` in `scene_right_approach.xml` has `euler="-2.0 0 0"`. This is a ~115-degree tilt from horizontal. The internal camera has `euler="3.14 0 0"` (180-degree flip around X). The D435 visual mesh also has `euler="0 0 3.14159"`. Compared to the working `scene_right_dynamic.xml` which uses `euler="-1.67079632679 0 0"` (~95.7 degrees), the approach scene's body euler is different. The camera needs a 180-degree Z rotation on the body to face the right direction.

### Issue 2: No interactive GUI
The launch file `mia_hand_system_interface_launch.py` always uses the headless `mia_hand_mujoco/SystemInterface` plugin (which uses `Simulator` - the simple GLFW window without Scene/Motion control). The `InteractiveSimulator` (with full GUI, Scene Control, Motion Control, Grasp Planner) is a separate plugin (`mia_hand_mujoco/InteractiveSystemInterface`) that is never selected by this launch file. The `hardware_plugin` parameter defaults to `mia_hand_mujoco/SystemInterface`.

### Issue 3: Hand proximity to ball
The hand (palm_r) is at `pos="-0.1 0 0.2"` (from `mia_hand_right_grasp_frame.xml:51`). The sphere is at `pos="-0.1 -0.0499124 0.3100398"`. The distance is roughly `sqrt(0^2 + 0.05^2 + 0.11^2) ≈ 0.12m`, which matches the `preshaping_distance` threshold. This is by design per the XML comment ("The hand starts ~20 cm behind the object").

### Issue 4: "No hand pose received yet"
This is the critical bug. The `approach_controller_node.py` subscribes to `/mujoco/hand_pose` (line 39, default). This topic is published by `InteractiveSystemInterface::read()` at ~10 Hz (line 354-374 of `interactive_system_interface.cpp`). However, the launch file uses the headless `SystemInterface` plugin which does NOT publish any pose topics - it only handles joint states. So `/mujoco/hand_pose` is never published, and the approach controller never receives a hand pose.

The `mujoco_scene_state_publisher_node.py` subscribes to `/mujoco/hand_pose` for base pose tracking (line 274-277), but it doesn't publish it - it consumes it. The scene state publisher does compute poses internally from its shadow MuJoCo model, but only publishes depth images, point clouds, and TF transforms - not the raw hand/object/camera poses.

## Implementation Plan

- [ ] **Task 1. Fix camera Z-rotation in `scene_right_approach.xml`**
  Add `3.14159` to the Z component of the `depth_cam_body` euler, changing `euler="-2.0 0 0"` to `euler="-2.0 0 3.14159"` to rotate the camera 180 degrees around Z. This mirrors what the dynamic scene does relative to its camera orientation. Also update the D435 visual mesh euler to match (remove the separate Z-rotation since it's now on the body).
  File: `docker_ws/dev/mujoco/scenes/scene_right_approach.xml:47`

- [ ] **Task 2. Add pose publishing to the headless `SystemInterface`**
  The headless `SystemInterface` (`system_interface.cpp`) needs to publish hand/object/camera poses on `/mujoco/hand_pose`, `/mujoco/object_pose`, and `/mujoco/camera_pose` - just like the `InteractiveSystemInterface` does. However, the headless `Simulator` class doesn't track body poses or have scene control.
  
  The simpler approach: have the `mujoco_scene_state_publisher_node.py` publish the hand/object/camera poses it already computes internally. It already reads body positions from its shadow MuJoCo model in `on_depth_timer` and `on_tf_timer`. Add publishers for these poses.
  
  Files to modify:
  - `docker_ws/dev/mujoco/nodes/mujoco_scene_state_publisher_node.py` - Add publishers for hand_pose, object_pose, camera_pose
  - `docker_ws/mia_hand_mujoco/launch/mia_hand_system_interface_launch.py` - Wire the pose topics if needed

- [ ] **Task 3. Verify hand starting distance is acceptable**
  The hand-to-object distance is ~0.12m which equals the preshaping_distance threshold. This means the approach controller would immediately trigger preshaping. If the intent is to demonstrate the full approach trajectory, the hand should start further back. However, the XML comment says "~20 cm behind the object" which doesn't match the actual geometry (~12cm). Consider moving the hand or object to achieve the stated 20cm distance.
  File: `docker_ws/dev/mujoco/scenes/scene_right_approach.xml` (sphere position at line 35)

## Verification Criteria

- [ ] Camera depth image shows the hand and object (not the ceiling/wall behind)
- [ ] `ros2 service call /approach_controller/start std_srvs/srv/Trigger` returns `success=True`
- [ ] `ros2 topic echo /mujoco/hand_pose --once` returns a valid pose
- [ ] `ros2 topic echo /mujoco/object_pose --once` returns a valid pose

## Potential Risks and Mitigations

1. **Pose publishing frequency mismatch**: The scene state publisher runs at `depth_publish_hz` (default 5 Hz), which is lower than the interactive interface's ~10 Hz. This should be sufficient for the approach controller.
   Mitigation: The approach controller checks at `check_hz` (default 20 Hz), but only needs pose updates at a few Hz to work.

2. **Shadow model pose accuracy**: The scene state publisher uses a "no-plugin" fallback model. Body poses are set from the same initial XML, so they should match. However, if the headless simulator moves bodies (which it currently can't since it has no scene control), the shadow model won't track.
   Mitigation: For the headless case, bodies don't move, so initial poses from XML are correct.

## Alternative Approaches

1. **Add pose tracking to headless Simulator**: Could add body ID lookups and pose publishing to the headless `Simulator`/`SystemInterface` C++ classes. This is more work but architecturally cleaner. Rejected for now because the Python scene state publisher already has the model loaded and the infrastructure.

2. **Make the launch file use InteractiveSystemInterface for approach scene**: Could default to the interactive plugin when `scene:=approach`. Rejected because the interactive simulator requires a display (GLFW window), which may not be available in headless/Docker environments.
