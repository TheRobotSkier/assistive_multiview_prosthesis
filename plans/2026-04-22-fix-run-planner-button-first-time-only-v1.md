# Diagnose: "Run Planner" Button Only Works the First Time

## Objective

Diagnose and plan a fix for the issue where the "Run Planner" button in the MuJoCo GUI only returns a valid grasp on the first click. Subsequent clicks return an open grasp (no collisions detected) unless the object is moved first.

## Root Cause Analysis

### The Pipeline (in order)

1. User clicks "Run Planner" button in `PlannerGuiSimulator` GUI
2. `launch_planner()` sets `planner_request_pending_ = true` and `planner_running_ = true`
3. In `PlannerGuiSystemInterface::read()` (called every control cycle), `consume_planner_request()` fires `trigger_preshaping_service()`
4. A ROS service call is made to `/grasp_preshaping/compute_grasp`
5. `PreshapingServiceBridgeNode::try_handle_direct_request()` copies the latest pose, twist, and point cloud from its subscriptions
6. The Rust `compute_from_request()` builds a TSDF from the point cloud, predicts ROI, prunes the cloud, scores grasps

### Identified Root Cause: The point cloud is stale after the first grasp

The preshaping bridge node subscribes to `/segmented_object_cloud` (`preshaping_service_bridge_node.cpp:72-78`). This cloud is produced by `MujocoSceneStatePublisher::on_depth_timer()` which runs at ~5 Hz and publishes the segmented object cloud.

**The critical issue is in the point cloud filtering** (`mujoco_scene_state_publisher_node.py:586-619` — `filter_points_by_target_geom`). It uses `mujoco.mj_ray()` ray casting to determine which depth pixels belong to the target geom (`target_sphere`). After the first grasp computation:

1. The preshaping node publishes joint commands that **close the hand** (thumb/index/mrl closure values)
2. These joint commands flow through `ros2_control` → `PlannerGuiSystemInterface::write()` → `PlannerGuiSimulator::set_jnt_pos()` → MuJoCo simulation
3. The **shadow MuJoCo model** in `MujocoSceneStatePublisher` also receives these joint states via `/joint_states` subscription
4. On the next depth timer tick, `on_depth_timer()` calls `mj_forward()` on the shadow model with the **closed hand** joint positions
5. When `filter_points_by_target_geom()` casts rays from the camera through the depth buffer, the **closed hand geometry now occludes the target object**
6. `mj_ray()` returns the hand geom ID instead of the target geom ID for those rays
7. The resulting point cloud is **empty or nearly empty** (all points filtered out because they don't match `target_geom_id`)
8. On the next "Run Planner" click, `compute_from_request()` receives an empty or minimal point cloud
9. `prune()` may return empty → "No points in ROI" error, OR the TSDF is too sparse for collision detection → no collisions found → open grasp returned

**Why moving the object fixes it**: Moving the object changes the scene geometry so rays can again reach the target from some angles, producing enough points for a valid TSDF.

### Evidence Trail

| File | Lines | What happens |
|------|-------|-------------|
| `preshaping_service_bridge_node.cpp` | 318-321 | Publishes joint commands that close the hand |
| `mujoco_scene_state_publisher_node.py` | 461-467 | Receives joint states (including closed hand) |
| `mujoco_scene_state_publisher_node.py` | 550-568 | Applies closed-hand joint positions to shadow model |
| `mujoco_scene_state_publisher_node.py` | 475 | Calls `mj_forward()` with closed hand |
| `mujoco_scene_state_publisher_node.py` | 596-617 | Ray-casts through depth buffer; hand geoms now occlude target |
| `mujoco_scene_state_publisher_node.py` | 617 | Only keeps points where `geomid[0] == target_geom_id` — most get filtered |
| `c_api.rs` | 362-365 | `prune()` returns empty → "No points in ROI" |

## Implementation Plan

- [ ] **Task 1: Open the hand before publishing the point cloud for grasp planning.** In `MujocoSceneStatePublisher::on_depth_timer()`, before calling `mj_forward()` and generating the segmented point cloud, reset the finger joint positions (j_thumb_fle, j_index_fle, j_mrl_fle) to their open (0.0) values in the shadow model's `data.qpos`. This ensures the depth rendering and ray-casting see an open hand that doesn't occlude the target object. Save the original joint values and restore them after the point cloud is generated so the TF and body pose outputs still reflect the actual simulation state.

- [ ] **Task 2: Alternative approach (if Task 1 is too invasive) — Disable hand geoms during ray-casting.** In `filter_points_by_target_geom()`, set `self.geomgroup` to exclude hand geom groups before calling `mujoco.mj_ray()`. MuJoCo's `geomgroup` parameter controls which geom groups participate in ray-casting. If the hand is in a different geom group than the target, set the hand's group bit to 0 so the rays pass through the hand and only detect the target. Restore the original `geomgroup` after filtering. This avoids modifying joint positions and is more targeted.

- [ ] **Task 3: Verify the fix.** Confirm that:
  1. Clicking "Run Planner" multiple times in succession returns valid grasps each time
  2. The hand still opens/closes correctly after grasp commands
  3. TF transforms and body poses remain accurate
  4. Moving the object still works correctly

## Verification Criteria

- [ ] Clicking "Run Planner" 3+ times in succession without moving the object produces valid grasp results each time
- [ ] The point cloud published on `/segmented_object_cloud` contains points even when the hand is closed
- [ ] No regression in grasp quality when the hand is in its default open position
- [ ] TF and body pose outputs remain correct

## Potential Risks and Mitigations

1. **Risk: Excluding hand geoms from ray-casting may cause rays to pass through the hand and hit objects behind it, adding incorrect points to the cloud.**
   Mitigation: The depth image itself still only contains depths from the actual rendering (which includes the hand). The ray-casting is only used for *labeling* which depth pixels belong to the target geom. So excluding the hand from ray-casting just means we correctly label the target points that are behind where the hand would be — but since the depth image was rendered with the hand, those pixels would already have hand-depth values, not target-depth values. Actually, this means the depth image AND the ray-casting need to be consistent. The better approach is Task 1 (open the hand for the shadow model's depth rendering).

2. **Risk: Temporarily modifying joint positions in the shadow model could cause race conditions if TF timer fires simultaneously.**
   Mitigation: Both `on_depth_timer()` and `on_tf_timer()` hold `self.state_lock`, so they cannot run concurrently. The joint position modification is contained within the lock.

3. **Risk: Opening the hand for the shadow model may cause visual artifacts in the depth image if someone is viewing it.**
   Mitigation: The depth image is an internal product used only for point cloud generation, not displayed to users. The actual MuJoCo GUI runs its own separate simulation instance.

## Recommended Approach

**Task 1 is the recommended approach.** The cleanest fix is to reset finger joints to open (0.0) in the shadow model before rendering depth and generating the segmented point cloud. This ensures:
- Depth rendering sees an open hand → target object is fully visible
- Ray-casting sees an open hand → all target pixels are correctly labeled
- The TSDF is built from a complete point cloud of the target
- TF and body poses can be computed in a separate pass with the actual joint values (which already happens in `on_tf_timer()`)

The implementation would modify `on_depth_timer()` in `mujoco_scene_state_publisher_node.py` to temporarily zero out the finger flexion joints before `mj_forward()` and the depth/cloud pipeline, then restore them before returning.
