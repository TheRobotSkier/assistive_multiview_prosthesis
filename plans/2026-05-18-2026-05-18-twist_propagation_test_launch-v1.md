# Plan: Twist Propagation Test Launch File

## Objective

Create a single launch file that brings up all nodes needed to test twist propagation with the Jetson arm camera streaming at `/arm/d435i_arm/points_marker_map`.

## What the launch file needs to start

| Node | Package | Why |
|------|---------|-----|
| Segmentation bridge | `segmentation_bridge` | Twist propagation triggers segmentation on hit; bridge calls inference server |
| Grasp preshaping service | `grasp_preshaping` | Twist propagation calls `/grasp_preshaping/compute_grasp` after segmentation completes |
| Twist propagation | `twist_propagation` | The node under test |
| RViz | `rviz2` | Visualization with `twist_propagation.rviz` |

## What the launch file does NOT start

- Camera nodes (already running on Jetson)
- Hand pose publisher (no tracked hand — `/hand_pose` must be published manually or by a test script)
- Pipeline manager, proximity controller, force controller (not needed for this test)

## Key configuration

- **Input cloud topic**: `/arm/d435i_arm/points_marker_map` (Jetson arm camera)
- **Cloud frame**: `marker_map` (matches `twist_propagation.rviz` fixed frame)
- **Segmentation input**: remap `/segmentation/input_cloud` → `/arm/d435i_arm/points_marker_map`
- **Twist propagation active**: `true` (start immediately, no need to call activate service)

## Implementation Tasks

- [ ] Create `src/prosthesis_launch/launch/twist_propagation_test.launch.py`
  - Declare launch arguments:
    - `input_cloud_topic` (default: `/arm/d435i_arm/points_marker_map`)
    - `active` (default: `true`)
    - `rviz` (default: `true`)
    - `inference_url` (default: `http://127.0.0.1:5678`)
  - Nodes:
    1. **Segmentation bridge** — remap `/segmentation/input_cloud` to `input_cloud_topic`
    2. **Grasp preshaping service** — no special params
    3. **Twist propagation** — params: `input_cloud_topic`, `active:=true`, rest from defaults
    4. **RViz** — config: `rviz/twist_propagation.rviz` (already has fixed frame `marker_map`)
  - Follow the same pattern as `grasp_test.launch.py` (OpaqueFunction for conditional logic)

- [ ] Rebuild: `make build` (colcon needs to pick up the new launch file)

- [ ] Test: `ros2 launch prosthesis_launch twist_propagation_test.launch.py`

## Usage after creating

```bash
# Shell 1: Start the pipeline
make shell
source /prosthesis_ws/install/setup.bash
ros2 launch prosthesis_launch twist_propagation_test.launch.py

# Shell 2: Publish fake hand pose moving toward an object
make shell
source /prosthesis_ws/install/setup.bash
for z in $(seq 0.80 -0.01 0.20); do
  ros2 topic pub --once /hand_pose geometry_msgs/msg/PoseStamped \
    "{header: {frame_id: 'marker_map'}, 
      pose: {position: {x: 0.0, y: 0.0, z: $z}, orientation: {w: 1.0}}}"
  sleep 0.1
done
```

## Verification

- [ ] RViz opens with `twist_propagation.rviz` config
- [ ] Scene pointcloud visible from `/arm/d435i_arm/points_marker_map`
- [ ] Twist propagation status shows `"active": true, "state": "IDLE", "reason": "waiting_for_poses"`
- [ ] Publishing `/hand_pose` → trajectory line appears in RViz
- [ ] When trajectory intersects cloud → green hit marker + log message
- [ ] Segmentation triggered → segmented cloud appears
- [ ] Grasp preshaping service called → finger closures published

## Risks

| Risk | Mitigation |
|------|-----------|
| `/hand_pose` frame mismatch | Use `marker_map` as frame_id (same as cloud) |
| Segmentation server not running | Start separately: `make up-grasp-test` or start segmentation container |
| Cloud topic frame not `marker_map` | Check with `ros2 topic echo` — adjust if needed |
| RViz fixed frame wrong | Already `marker_map` in config, matches the cloud topic |
