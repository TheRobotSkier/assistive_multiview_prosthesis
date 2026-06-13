# Localization Rework V6 — Phase 2: ROS Nodes (Keyframe Buffer + MobileSAM Server)

**Date:** 2026-06-13
**Parent plan:** `plans/2026-06-13-localization-rework-plan-v6.md`
**Depends on:** Phase 0 (environment), Phase 1 Task A (`sensor_fusion_msgs`), Phase 1 Task B (`cloud_utils`)
**Blocks:** Phase 3 Task A (TSDF fusion needs keyframe buffer)
**Estimated time:** 3–4 days
**Parallelism:** 2 agents (Tasks A and B are fully independent)

---

## Objective

Build two independent components:
- **A:** `keyframe_buffer` ROS node — spatial-gated storage of per-camera keyframes with the `GetKeyframesInROI` service. This is the data backbone for TSDF fusion.
- **B:** MobileSAM inference server — a standalone Docker/Flask service that takes an RGB image + 2D click and returns a 2D binary mask. This replaces InterObject3D.

Task B has **zero ROS dependencies** and can be developed/tested entirely on the host. Task A is a ROS2 node that wraps the pure-logic `cloud_utils` from Phase 1.

---

## Task A: `keyframe_buffer` ROS Node

**Agent:** 1 (~2.5 days)
**Depends on:** Phase 1 Task B (`cloud_utils` must be built and tested)
**Blocks:** Phase 3 Task A (TSDF fusion)

### A.1 Node structure

The package skeleton was created in Phase 1 Task B (`src/keyframe_buffer/`). Now add the ROS node and keyframe dataclass:

```
src/keyframe_buffer/
├── keyframe_buffer/
│   ├── __init__.py
│   ├── cloud_utils.py           # DONE in Phase 1
│   ├── keyframe.py              # NEW: Keyframe dataclass
│   └── keyframe_buffer_node.py  # NEW: Main ROS node (~400 lines)
├── srv/
│   └── GetKeyframesInROI.srv    # NEW: service definition
├── test/
│   ├── test_cloud_utils.py      # DONE in Phase 1
│   └── test_keyframe_buffer.py  # NEW: node logic tests
├── resource/
│   └── keyframe_buffer
├── setup.py                     # UPDATE: add node entry point + srv
└── package.xml                  # UPDATE: add service deps
```

### A.2 Keyframe dataclass

Implement `keyframe_buffer/keyframe.py` (V6 plan §6.2):

```python
@dataclass
class Keyframe:
    timestamp: float
    camera_id: str              # "head" or "arm"
    cloud_xyz: np.ndarray       # (N, 3) or (H, W, 3)
    cloud_rgb: np.ndarray       # (N, 3) or (H, W, 3)
    image: np.ndarray           # (H, W, 3) uint8 RGB
    K: np.ndarray               # (3, 3) intrinsics
    pose: np.ndarray            # (4, 4) T_marker_map->camera
    organized: bool             # True if cloud is (H, W, 3)
```

- [ ] Implement the dataclass with `__slots__` or dataclass for memory efficiency.
- [ ] Add a `memory_mb` property that returns the approximate memory footprint (sum of array nbytes). Used for diagnostics.

### A.3 Service definition

Create `srv/GetKeyframesInROI.srv`:

```
# Request
geometry_msgs/Point center
float64 radius
---
# Response
sensor_fusion_msgs/Keyframe[] keyframes    # OR a custom serialized format
int32 count
```

> **Design note:** Serializing full numpy arrays + images through a ROS service is bandwidth-heavy. Consider two options:
> 1. **In-process:** If TSDF fusion runs in the same process (composed), use a direct Python call, not a service.
> 2. **Service with PointCloud2:** Return each keyframe as a `sensor_msgs/PointCloud2` + `sensor_msgs/Image` + `geometry_msgs/Pose` + `sensor_msgs/CameraInfo` bundle.
>
> **Recommendation:** Start with option 2 (service) for clean process isolation, but design the node so the core `get_in_roi()` method is a pure-Python function that returns `list[Keyframe]`. The service handler just serializes. This keeps it testable.

- [ ] Decide on the service message format. If a custom `Keyframe.msg` is needed, define it in `sensor_fusion_msgs` (coordinate with Phase 1 Task A owner) or inline in this package.
- [ ] Write the `.srv` file.
- [ ] Update `CMakeLists.txt`/`setup.py` to generate the service. Since this is an `ament_python` package, service generation requires `rosidl`. If the package is pure-Python, you may need to convert to `ament_cmake` or define the service in `sensor_fusion_msgs` instead. **Check the existing convention** — `src/pointcloud_fusion` is `ament_python` and has no custom services, so there may be no precedent. If `ament_python` can't generate services cleanly, define `GetKeyframesInROI.srv` in `sensor_fusion_msgs` (Phase 1 Task A package, which is `ament_cmake`).

### A.4 Main node implementation

Implement `keyframe_buffer_node.py` following V6 plan §6.2:

- [ ] **Parameters** (declare all with defaults from V6 §6.9 config):
  - `head_image_topic`, `arm_image_topic`
  - `head_cloud_topic`, `arm_cloud_topic`
  - `head_info_topic`, `arm_info_topic`
  - `head_pose_topic: "/gtsam/head_pose"`, `arm_pose_topic: "/gtsam/arm_pose"`
  - `max_keyframes_per_camera: 50`
  - `spatial_gate_translation_m: 0.10`
  - `spatial_gate_rotation_deg: 15.0`

- [ ] **Independent subscriptions (CRITICAL):**
  - Subscribe to each camera's cloud, image, camera_info, and pose **independently** — do NOT use `ApproximateTimeSynchronizer`.
  - Maintain per-camera "latest" buffers for image, cloud, camera_info, and pose.
  - When a new cloud arrives (the slowest signal), check if a pose is available within 50ms; if so, evaluate the spatial gate and potentially create a keyframe.

- [ ] **Cloud organization auto-detection (V6 §5.5 — mandatory):**
  ```python
  if self._cloud_organized is None:
      self._cloud_organized = (msg.height > 1)
      self.get_logger().info(f"{camera_id} cloud: height={msg.height} -> "
                             f"{'ORGANIZED' if self._cloud_organized else 'UNORGANIZED'}")
  ```

- [ ] **Spatial gate** (per camera, independent):
  - Translation: reject if `||t_new - t_last|| < spatial_gate_translation_m`
  - Rotation: reject if `angle_between_quaternions(q_new, q_last) < spatial_gate_rotation_deg`
  - Use `se3_helpers.angle_between_quaternions` from Phase 1 Task C.
  - Only accept if BOTH translation AND rotation thresholds are exceeded (i.e., enough motion has happened).

- [ ] **Keyframe creation:**
  - Parse PointCloud2 into `(N, 3)` xyz + `(N, 3)` rgb using `rosbags`-style struct unpacking or `sensor_msgs_py.point_cloud2` (check what's available in-container).
  - Store image as `(H, W, 3)` uint8.
  - Store pose as `(4, 4)` from the PoseWithCovariance message.
  - Create `Keyframe` dataclass, append to the per-camera ring buffer.

- [ ] **Eviction:** `collections.deque(maxlen=max_keyframes_per_camera)` per camera. Oldest dropped automatically.

- [ ] **Service handler** `get_in_roi`:
  - Iterate all keyframes in both cameras.
  - Keep those where `||keyframe.pose_translation - center|| < radius`.
  - Return the list.

- [ ] **Diagnostics publisher:** Publish a `diagnostic_msgs/DiagnosticArray` every 5s with:
  - Total keyframes per camera
  - Total memory in MB
  - Cloud organization mode

### A.5 Tests

- [ ] **`test/test_keyframe_buffer.py`** — test the pure-logic parts without rclpy:
  - **Spatial gate logic:** Create a mock buffer (plain class, not a Node), feed a sequence of poses, verify that keyframes are accepted/rejected correctly based on translation and rotation thresholds.
  - **Ring buffer eviction:** Fill beyond `max_keyframes`, verify oldest is dropped.
  - **ROI query:** Insert keyframes at known positions, query a region, verify correct subset returned.
  - **Memory accounting:** Create keyframes with known array sizes, verify `memory_mb` property.
  - These tests use the `Keyframe` dataclass and a stripped-down buffer class — no ROS needed. Run on host: `python3 -m pytest src/keyframe_buffer/test/test_keyframe_buffer.py -v`.

- [ ] **ROS integration smoke test** (in-container, uses mock data):
  - Launch the node with `mock.launch.py` providing fake cloud/image/pose publishers.
  - Verify the node logs the cloud organization detection.
  - Call the `GetKeyframesInROI` service and verify it responds.

### A.6 Build and verify

- [ ] `make build-pkg PKG=keyframe_buffer` succeeds.
- [ ] All tests pass.
- [ ] Node launches without error in the container with mock data.

---

## Task B: MobileSAM Inference Server

**Agent:** 1 (independent, ~1.5 days)
**Depends on:** Nothing (pure Python/Docker, no ROS)
**Blocks:** Phase 3 Task A (TSDF fusion calls this server)

### B.1 Dockerfile

Create `docker/Dockerfile.segmentation_v2` (V6 plan §6.4):

```dockerfile
FROM python:3.10-slim

WORKDIR /app

# Install system deps for OpenCV
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1 libglib2.0-0 && rm -rf /var/lib/apt/lists/*

# Install Python deps
RUN pip install --no-cache-dir \
    torch torchvision --index-url https://download.pytorch.org/whl/cpu \
    flask numpy opencv-python-headless Pillow

# MobileSAM
RUN pip install --no-cache-dir mobile-sam

# Copy server code
COPY scripts/mobile_sam_server.py /app/

# Pre-download weights (or mount at runtime)
ENV SAM_WEIGHTS=/weights/mobile_sam.pt

EXPOSE 5679

CMD ["python3", "/app/mobile_sam_server.py"]
```

- [ ] Write `docker/Dockerfile.segmentation_v2`.
- [ ] **GPU variant:** If the laptop has the GTX 3050 and we want GPU inference, create a `Dockerfile.segmentation_v2_gpu` using `nvidia/cuda` base + `torch` GPU wheels. For now, CPU-only is the safe default (MobileSAM is fast enough on CPU for burst-mode grasp-time inference).

### B.2 Inference server

Create `scripts/mobile_sam_server.py` (V6 plan §6.4):

```python
"""
MobileSAM inference server.
POST /segment_2d with JSON: {"image_b64": "...", "click_x": int, "click_y": int, "dilation_px": int}
Returns JSON: {"mask": [[bool,...],...], "height": H, "width": W}
"""
```

- [ ] **Model loading:** Load MobileSAM at startup. Log the cold-start time (V6 validation gate: < 5s).
- [ ] **Endpoint `/segment_2d`:**
  - Accept base64-encoded RGB image + click coordinates + optional dilation.
  - Run `sam.predict(point_coords=[[x, y]], point_labels=[1])`.
  - Dilate the mask by `dilation_px` (default 15) using `cv2.dilate`.
  - Return the mask as a base64-encoded PNG or JSON 2D array.
- [ ] **Endpoint `/health`:** Return `{"status": "ready"}` — used by TSDF fusion to check availability before calling.
- [ ] **Logging:** Log inference time per request (V6 validation gate: < 300ms for burst of 20).

### B.3 Weights

- [ ] Download MobileSAM weights (`mobile_sam.pt` or `vit_b` variant). Place in `models/` or a mounted volume at `/weights/`.
- [ ] Document the download command in the Dockerfile comments.

### B.4 Tests

- [ ] **Unit test (host, no Docker):** Load a test image (use a synthetic or sample image), call the segmentation function directly (not via HTTP), verify the mask shape matches the image dimensions and is binary.
- [ ] **Integration test (Docker):**
  ```bash
  docker build -t mobile_sam_server -f docker/Dockerfile.segmentation_v2 .
  docker run -d -p 5679:5679 --name sam_test mobile_sam_server
  # Wait for health
  curl http://localhost:5679/health
  # Send a test request with a sample image
  python3 scripts/test_sam_server.py  # sends a sample image + click
  ```
  Verify the mask is returned and has the expected shape.
- [ ] **Cold-start timing:** Measure time from container start to `/health` returning ready. Must be < 5s.

### B.5 Docker compose integration

- [ ] Add the MobileSAM service to `docker/docker-compose.yml`:
  ```yaml
  mobile_sam:
    build:
      context: .
      dockerfile: docker/Dockerfile.segmentation_v2
    ports:
      - "5679:5679"
    volumes:
      - ./models:/weights:ro
  ```
  (Coordinate with Phase 0 — the compose file may already be structured for this.)

---

## Verification Gate

Phase 2 is complete when ALL of the following are true:
1. `keyframe_buffer` node builds and launches in-container with mock data.
2. Cloud organization auto-detection logs correctly (UNORGANIZED expected).
3. `GetKeyframesInROI` service responds with keyframes in the requested region.
4. Spatial gate correctly filters keyframes (verified by unit tests).
5. Ring buffer evicts oldest keyframes at capacity.
6. Memory diagnostics publisher reports < 350 MB at 100 keyframes.
7. MobileSAM server builds and responds to `/health` within 5s of startup.
8. MobileSAM server returns a valid binary mask for a test image + click within 300ms.
9. All unit tests pass: `python3 -m pytest src/keyframe_buffer/test/ -v`.

---

## Dependency Graph

```
Phase 1 Task B (cloud_utils) ──► Task A (keyframe_buffer node)
                                    │
                                    └──► Phase 3 Task A (TSDF fusion)

(nothing) ──────────────────► Task B (MobileSAM server)
                                    │
                                    └──► Phase 3 Task A (TSDF fusion)
```

Tasks A and B are fully independent and can proceed in parallel.

---

## Risk Notes

- **Service generation in ament_python:** ROS2's `ament_python` does not natively generate `.srv` files. If this becomes a blocker, define `GetKeyframesInROI.srv` in `sensor_fusion_msgs` (the CMake-based message package from Phase 1 Task A) instead. This is the cleaner approach and keeps the keyframe_buffer package pure-Python.
- **PointCloud2 parsing:** Parsing PointCloud2 into numpy is verbose. Check if `sensor_msgs_py` (`pip install sensor-msgs-py` or the ROS-provided `sensor_msgs_py` module) is available in-container. If not, use `rosbags` library's deserializer or manual `struct.unpack`. The existing `pointcloud_fusion` node must already do this — check `src/pointcloud_fusion/pointcloud_fusion/` for the pattern to reuse.
- **MobileSAM on CPU:** CPU inference for a single point prompt should be < 200ms. If it's slower, the TSDF fusion burst (20 keyframes) could exceed the 300ms budget. Mitigation: batch the inference or reduce keyframe count.
- **Memory:** The keyframe buffer holds raw numpy arrays. At 100 keyframes × ~2.7 MB = ~270 MB. The diagnostics publisher must confirm this stays bounded. If point counts are larger on the live system, adjust `max_keyframes_per_camera` down.