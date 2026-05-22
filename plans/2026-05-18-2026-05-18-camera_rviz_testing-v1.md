# Plan: Testing Grasp Preshaping with a Camera and RViz

## Context

You have a single RealSense D435 camera connected to your local machine and want to visually test the grasp preshaping pipeline through RViz. The system already has multiple launch paths, but they're designed for either:
- **Mock mode** (no camera, synthetic clouds) — `mock.launch.py`
- **Dual cameras on Jetson** (head + arm) — `digital_twin.launch.py`, `grasp_test.launch.py camera:=true`
- **Dual cameras via `two_d435.launch.py`** (hardcoded serials for old D435s)

None of these work out-of-the-box for a **single camera on the local machine**. The closest option is `grasp_test.launch.py` with `camera:=true`, but it expects the `sensor_fusion_bringup/dual_d435i.launch.py` which tries to start **two** cameras with specific serials from `d435i_cameras.yaml`.

## Recommended Approach: Use `grasp_test.launch.py` in Mock Mode + Manual Camera

The simplest path that gets you a live pointcloud AND the full grasp pipeline:

### Step-by-step

1. **Start the segmentation server** (if not already running):
   ```
   make up-grasp-test
   ```
   This starts the `grasp_test` container (which includes segmentation + pipeline). But by default it uses `camera:=true` and expects dual cameras.

2. **Alternative: Start mock pipeline + separate camera node**

   Actually, the cleanest approach is:

   **Terminal 1 — Main pipeline (mock cloud, no camera):**
   ```bash
   # Inside the prosthesis container
   ros2 launch prosthesis_launch grasp_test.launch.py camera:=false gui:=true
   ```
   This gives you: preshaping service, proximity controller, pipeline manager, segmentation bridge, RViz with `grasp_test.rviz`, joint_state_publisher_gui.

   **Terminal 2 — Single camera node:**
   ```bash
   # Inside the same container (make shell from another terminal)
   ros2 launch sensor_fusion_bringup single_d435i.launch.py camera_key:=head
   ```
   This starts just the head camera using the serial from `d435i_cameras.yaml`. It publishes to `/head/d435i_head/depth/color/points`.

   **But wait** — the segmentation bridge in `grasp_test.launch.py camera:=false` subscribes to `/camera/depth/color/points` (the mock topic). You'd need to either:
   - Remap the segmentation input to the camera topic, OR
   - Use `camera:=true` but only plug in one camera

3. **Best option: Modify the grasp_test launch to support single-camera mode**

   Actually, let me reconsider. The simplest working approach:

   **Option A: `grasp_test.launch.py camera:=true` with one camera**
   
   The `dual_d435i.launch.py` will fail on the second camera (arm) if it's not connected, but the head camera will still start. The arm camera node will just error and the pipeline will continue without it. The pointcloud merger will only get one cloud. This is the path of least resistance.

   **Option B: Use `digital_twin.launch.py camera:=true` with one camera**
   
   Same situation — dual launch, one camera fails, but the pipeline continues.

   **Option C: Manual approach — full control**

   This is the most reliable for a single-camera test session.

---

## Plan: Option C — Manual Single-Camera Test (Recommended)

### Prerequisites
- One RealSense D435 connected via USB
- Docker/Podman running
- X11 display working (RViz needs it)
- The `prosthesis` image built (`make build`)

### Implementation Tasks

- [ ] **Task 1. Verify the camera is visible inside the container**
  Run `make shell` then:
  ```bash
  rs-enumerate-devices --short
  ```
  Should list your D435. If not, the container may need `--device /dev/bus/usb` or `privileged: true` (already set in docker-compose.yml).

- [ ] **Task 2. Start the segmentation server**
  ```bash
  # From host
  cd docker && podman-compose --profile segmentation up segmentation -d
  # Or: make up-grasp-test (starts both segmentation + grasp_test)
  ```

- [ ] **Task 3. Start the main pipeline container**
  ```bash
  make up
  make shell
  ```

- [ ] **Task 4. Inside the container, launch the single camera**
  ```bash
  source /prosthesis_ws/install/setup.bash
  ros2 launch sensor_fusion_bringup single_d435i.launch.py camera_key:=head
  ```
  This publishes:
  - `/head/d435i_head/depth/color/points` (pointcloud)
  - `/head/d435i_head/color/image_raw` (RGB image)
  - TF frames: `d435i_head_depth_optical_frame`, etc.

  **If your camera serial doesn't match** the one in `d435i_cameras.yaml` (`_336222071386`), you need to either:
  - Update the serial in `src/sensor_fusion_bringup/config/d435i_cameras.yaml`
  - Or launch the realsense node directly without a serial filter:
    ```bash
    ros2 launch realsense2_camera rs_launch.py \
      camera_namespace:=head camera_name:=d435i_head \
      pointcloud.enable:=true align_depth.enable:=true \
      depth_module.depth_profile:=640x480x30 rgb_camera.color_profile:=640x480x30
    ```

- [ ] **Task 5. In a second shell, launch the pipeline nodes**
  ```bash
  make shell
  source /prosthesis_ws/install/setup.bash
  
  # Static TF to root the camera at world origin
  ros2 run tf2_ros static_transform_publisher \
    0 0 0 0 0 0 1 d435i_head_depth_optical_frame world &

  # Segmentation bridge
  ros2 run segmentation_bridge segmentation_ros2_node \
    --ros-args -p inference_url:=http://127.0.0.1:5678 \
    --remap /segmentation/input_cloud:=/head/d435i_head/depth/color/points &

  # Grasp preshaping service
  ros2 run grasp_preshaping preshaping_service_bridge_node &

  # Wait, then launch RViz
  sleep 3
  rviz2 -d /prosthesis_ws/rviz/prosthesis.rviz
  ```

- [ ] **Task 6. In RViz, configure the display**
  - Set **Fixed Frame** to `world` (or `d435i_head_depth_optical_frame`)
  - Add **PointCloud2** display → topic: `/head/d435i_head/depth/color/points`
  - Add **PointCloud2** display → topic: `/segmentation/object_cloud` (orange, larger points)
  - Add **MarkerArray** display → topic: `/grasp/markers`
  - Add **TF** display to see frames

- [ ] **Task 7. Trigger segmentation**
  Use the RViz **Publish Point** tool to click on the object in the pointcloud. This publishes to `/clicked_point`, which the segmentation bridge uses as a seed.
  
  Or use the segmentation click topics directly:
  ```bash
  ros2 topic pub --once /segmentation/click_positive geometry_msgs/msg/PointStamped \
    "{header: {frame_id: 'd435i_head_depth_optical_frame'}, point: {x: 0.3, y: 0.0, z: 0.3}}"
  ```

- [ ] **Task 8. Trigger grasp computation**
  Once you have a segmented cloud on `/segmentation/object_cloud`:
  ```bash
  ros2 service call /grasp_preshaping/compute_grasp grasp_preshaping/srv/ComputeGrasp \
    "{hand_pose: {header: {frame_id: 'world'}, pose: {position: {x: 0.0, y: 0.0, z: 0.5}, orientation: {w: 1.0}}}}"
  ```

- [ ] **Task 9. Verify the output**
  Check the response and the published topics:
  ```bash
  ros2 topic echo /grasp_preshaping/target_finger_closures --once
  ros2 topic echo /grasp_preshaping/wrist_pose --once
  ros2 topic echo /grasp_preshaping/grasp_type --once
  ```

---

## Simpler Alternative: Use `make up-grasp-test` with Camera Passthrough

If you want the full automated pipeline:

- [ ] **Task A. Ensure the grasp_test container has USB access**
  The `docker-compose.yml` already sets `privileged: true` for `grasp_test`, so USB devices should be accessible.

- [ ] **Task B. Update camera serial in config**
  Check your camera serial: `rs-enumerate-devices --short` on the host.
  Update `src/sensor_fusion_bringup/config/d435i_cameras.yaml` head serial to match.
  Rebuild: `make build`

- [ ] **Task C. Start with camera enabled**
  ```bash
  # This runs: ros2 launch prosthesis_launch grasp_test.launch.py camera:=true gui:=true
  make up-grasp-test
  make logs-grasp-test
  ```
  The arm camera will fail (not connected), but the head camera should start. The pointcloud merger will only have one input but should still pass it through.

- [ ] **Task D. Click to segment in RViz**
  Use the "Publish Point" tool in RViz to click on the object. The segmentation bridge will process it and publish the segmented cloud.

---

## Simplest Possible Test: Just Camera + RViz (No Pipeline)

If you just want to see the camera pointcloud in RViz first to verify the camera works:

```bash
# Terminal 1: Start container
make up && make shell

# Inside container:
source /prosthesis_ws/install/setup.bash

# Start camera (no serial filter — uses first found)
ros2 launch realsense2_camera rs_launch.py \
  pointcloud.enable:=true align_depth.enable:=true &

sleep 5

# Start RViz with the realsense config
rviz2 -d /prosthesis_ws/rviz/realsense_pointcloud.rviz
```

This uses `rviz/realsense_pointcloud.rviz` which is already configured for pointcloud viewing.

---

## Verification Criteria

- [ ] Camera pointcloud visible in RViz (colored, not just noise)
- [ ] Can click on object → segmented cloud appears (orange points)
- [ ] Grasp service call returns finger closures, wrist pose, grasp type
- [ ] Grasp markers visible in RViz (hand position + finger visualization)

## Potential Risks

1. **Camera serial mismatch**: The config has hardcoded serials. Fix: update `d435i_cameras.yaml` or launch without serial filter.
2. **USB permissions in container**: `privileged: true` should handle this, but check with `rs-enumerate-devices` inside the container.
3. **Segmentation server not running**: The grasp_test profile starts it, but standalone doesn't. Start it separately.
4. **RViz fixed frame mismatch**: If the camera TF frames don't match what RViz expects, nothing displays. Set fixed frame to `camera_link` or `d435i_head_depth_optical_frame`.
5. **Firmware version**: D435 needs recent firmware for ROS2 realsense driver. `rs-fw-update` to check.
