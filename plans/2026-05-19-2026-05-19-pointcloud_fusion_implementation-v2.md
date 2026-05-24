# Point Cloud Fusion Node — Final Implementation Plan

**Date:** 2026-05-19
**Status:** Ready for implementation
**Depends on:** Investigation of `jetson_docker` and `wt-x86-full-digital-twin` branches (completed)

---

## Investigation Summary

### Branch Findings

#### `jetson_docker` Branch

| Asset | Path | Relevance |
|---|---|---|
| **C++ frame transform node** | `docker_ws/multi_cam_localization/sensor_fusion_bringup/src/pointcloud_to_frame_node.cpp` | Transforms point cloud from camera optical frame to target frame via TF2, includes voxel grid downsampling (PCL). **Reference for C++ approach** — but we use Python. |
| **Implementation report** | `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_openvins_pointcloud_implementation_report_2026-05-14.md` | Documents OpenVINS-based point cloud pipeline. |
| **Validation report** | `docker_ws/multi_cam_localization/sensor_fusion_bringup/docs/d435i_color_pointcloud_marker_map_validation.md` | Validates color point cloud with marker map. |
| **Self-filter prompt** | `.agents/prosthesis_self_filter_pointcloud_prompt_2026-05-15.md` | Discusses self-filtering (hand removal) from point clouds. |
| **BBox corner transforms** | Present in branch | Transforms for corners of bounding box defined in arm frame. |

#### `wt-x86-full-digital-twin` Branch

| Asset | Path | Relevance |
|---|---|---|
| **World-frame merger** | `src/camera/camera/pointcloud_merger_node.py` | Enhanced version that fuses in **world frame** instead of cam1 frame. This is the most relevant reference. |
| **OpenVINS hand tracker** | `src/camera/camera/openvins_hand_tracker_node.py` | Enhanced hand tracker using OpenVINS (vs ChArUco-only on current branch). |
| **World frame fusion report** | `docs/world_frame_fusion_report.md` | Documents the world-frame fusion approach. |
| **Digital twin implementation report** | `docs/x86_full_digital_twin_implementation_report.md` | Full implementation documentation including bbox transforms and self-filtering. |
| **Enhanced digital twin launch** | `src/prosthesis_launch/launch/digital_twin.launch.py` | Launch file with bbox corner transforms integrated. |

### Decision Matrix

| Question | Decision | Rationale |
|---|---|---|
| **ICP needed?** | **No** | OpenVINS + ChArUco markers provide accurate TF chain. No ICP code exists in any branch. The `pointcloud_to_frame_node.cpp` on jetson_docker relies purely on TF2 transforms and reports good alignment. Adding ICP would introduce significant latency for marginal gain. |
| **Downsampling needed?** | **Yes — critical** | Segmentation inference server (`src/segmentation/nodes/inference_server.py:64-95`) processes ALL points through MinkowskiEngine with no internal subsampling. Single D435i: ~100K-200K valid depth points. Fused dual-camera: ~200K-400K points. Voxel downsampling to ~50K-80K points will dramatically reduce inference time without significant quality loss. |
| **Distance filtering?** | **Yes — 2m from wrist** | Remove points >2m from wrist. Reduces cloud size significantly (most of a 4m D435i range is irrelevant). Do **after** merging in world frame for simplicity. |
| **Hand/arm removal?** | **Yes** | Use AABB in arm frame (wrist_link). The jetson_docker branch has bbox corner transforms. The wt-x86 branch documents this approach. Define bbox in `wrist_link` frame, transform to world frame, remove interior points. |
| **Target frame?** | **`world`** | Both branches converge on world-frame fusion. The current branch uses `cam1_depth_optical_frame` which is wrong for proper dual-camera fusion — it biases toward cam1 and doesn't properly handle the case where cam2 has a better view. World frame is frame-agnostic. |
| **Extend or replace merger?** | **New package** | New `pointcloud_fusion` package in `src/pointcloud_fusion/`. Clean separation from the camera package. |

---

## Current Topic Flow (Critical for Integration)

The current pipeline in `digital_twin.launch.py` is:

```
cam1/cam2 depth/color/points
    → pointcloud_merger_node (camera pkg) → /fused_pointcloud
        → pointcloud_relay_node (camera pkg) → /segmentation/input_cloud
            → segmentation_bridge → /segmentation/object_cloud
                → twist_propagation_node (subscribes to both input_cloud AND object_cloud)
```

### Twist Propagation Subscriptions (from `src/twist_propagation/twist_propagation/twist_propagation_node.py:359-364`):

| Parameter | Default | Actual (from launch) | Purpose |
|---|---|---|---|
| `input_cloud_topic` | `/camera/depth/color/points` | `/head/d435i_head/depth/color/points` | Raw cloud for collision detection |
| `segmented_cloud_topic` | `/segmentation/object_cloud` | `/segmentation/object_cloud` | Segmented object cloud |
| `hand_pose_topic` | `/hand_pose` | `/hand_pose` | Hand pose from TF |
| `click_positive_topic` | `/segmentation/click_positive` | `/segmentation/click_positive` | Click seeds |

**Important:** Twist propagation's `input_cloud_topic` is currently set to the **raw cam1 cloud** (`/head/d435i_head/depth/color/points`) at `src/prosthesis_launch/launch/digital_twin.launch.py:286`, NOT the fused cloud. This means twist propagation currently only sees cam1's view for collision detection. After the fusion node is implemented, this should be changed to `/fused_pointcloud` so it gets the full dual-camera view.

### Segmentation Bridge Subscription (from `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:127-128`):

Subscribes to `/segmentation/input_cloud` (hardcoded). This is fed by the relay node.

### Relay Node (from `src/camera/camera/pointcloud_relay_node.py:15-16`):

Hardcoded: subscribes `/fused_pointcloud` → publishes `/segmentation/input_cloud`.

### New Topic Flow After Implementation

```
cam1/cam2 depth/color/points
    → pointcloud_fusion_node (NEW pkg) → /fused_pointcloud
        → pointcloud_relay_node (camera pkg, unchanged) → /segmentation/input_cloud
            → segmentation_bridge → /segmentation/object_cloud
        → twist_propagation_node (input_cloud_topic changed to /fused_pointcloud)
```

**Key changes needed:**
1. `digital_twin.launch.py:154-168`: Replace `pointcloud_merger_node` with `pointcloud_fusion_node` from new package
2. `digital_twin.launch.py:286`: Change `input_cloud_topic` for twist propagation from raw cam1 topic to `/fused_pointcloud`
3. `config/prosthesis_config.yaml:106`: Update `input_cloud_topic` default to `/fused_pointcloud`

---

## Implementation Plan

### Step 1: Create `pointcloud_fusion` Package Structure

**New directory:** `src/pointcloud_fusion/`

```
src/pointcloud_fusion/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── pointcloud_fusion
└── pointcloud_fusion/
    ├── __init__.py
    └── pointcloud_fusion_node.py
```

- [x] **1.1.** Create `src/pointcloud_fusion/package.xml` — ament_python package with dependencies: `rclpy`, `sensor_msgs`, `geometry_msgs`, `visualization_msgs`, `tf2_ros`, `tf2_sensor_msgs`, `std_msgs`, `numpy`
- [x] **1.2.** Create `src/pointcloud_fusion/setup.py` — with entry point `pointcloud_fusion_node = pointcloud_fusion.pointcloud_fusion_node:main`
- [x] **1.3.** Create `src/pointcloud_fusion/setup.cfg` — standard ament_python config
- [x] **1.4.** Create `src/pointcloud_fusion/resource/pointcloud_fusion` — empty marker file
- [x] **1.5.** Create `src/pointcloud_fusion/pointcloud_fusion/__init__.py` — empty

### Step 2: Implement `pointcloud_fusion_node.py`

**File:** `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`

#### Architecture

```
/cam1/depth/color/points  ──┐
                              ├──> [pointcloud_fusion_node] ──> /fused_pointcloud
/cam2/depth/color/points  ──┘          │
                                        ├── TF2: cam*_optical_frame → world
                                        ├── Merge both clouds in world frame
                                        ├── Distance filter (2m from wrist_link)
                                        ├── Hand/arm bbox removal (wrist_link frame AABB)
                                        └── Voxel downsampling (5mm grid)
```

#### Node Design

```python
class PointCloudFusionNode(Node):
    """
    Subscribes to two camera point clouds, transforms both to world frame,
    merges, filters, downsamples, and publishes a unified cloud.
    """

    # Parameters (from config)
    target_frame: str = "world"           # TF target frame
    cam1_topic: str                       # e.g., /head/d435i_head/depth/color/points
    cam2_topic: str                       # e.g., /arm/d435i_arm/depth/color/points
    arm_frame: str = "wrist_link"         # Frame for distance filtering & bbox
    max_distance: float = 2.0             # Max distance from arm (meters)
    voxel_size: float = 0.005             # Voxel downsampling grid size (meters)
    bbox_min: list[float]                 # AABB min in arm frame [x, y, z]
    bbox_max: list[float]                 # AABB max in arm frame [x, y, z]
    enable_downsampling: bool = True
    enable_distance_filter: bool = True
    enable_hand_removal: bool = True
```

#### Processing Pipeline (in order)

1. **Receive both clouds** — Use approximate time synchronizer (~100ms tolerance) to pair clouds. Fall back to single-cloud mode if only one arrives within tolerance.

2. **Transform both to world frame** — Use `tf2_ros.Buffer.transform()` on each `PointCloud2`. If TF is not yet available for a cloud, skip it (log warning).

3. **Concatenate** — Merge the two transformed clouds by concatenating the point data arrays. Both are now in world frame with identical field layout.

4. **Distance filter (2m from arm)** — Look up `wrist_link` position in world frame via TF2. Compute Euclidean distance from each point to wrist position. Remove points beyond `max_distance`. This is a simple numpy operation on the xyz data.

5. **Hand/arm bbox removal** — Define an AABB in `wrist_link` frame. Transform all remaining points into `wrist_link` frame (one matrix multiply using the TF). Remove points where all three axes fall within the bbox bounds. Transform is done once for the whole array.

6. **Voxel downsampling** — Apply voxel grid filter. For each voxel cell, keep the centroid point (average xyz and rgb). This reduces ~200K-400K points to ~50K-80K depending on voxel size. Use numpy-based implementation (no open3d dependency needed).

7. **Publish** — Pack the filtered, downsampled points back into a `PointCloud2` message with `frame_id = "world"` and publish on `/fused_pointcloud`.

#### Implementation Tasks

- [x] **2.1.** Implement cloud parsing — extract xyz (N,3) float32 and rgb (N,3) uint8 from PointCloud2 message, matching the approach in `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:51-77`
- [x] **2.2.** Implement TF2-based cloud transformation — transform PointCloud2 from camera optical frame to world frame using `tf2_ros.Buffer.transform()`. Handle TransformException gracefully with logging.
- [x] **2.3.** Implement cloud concatenation — merge two PointCloud2 messages that share the same frame_id and field layout by concatenating their raw byte data. Update width and row_step fields.
- [x] **2.4.** Implement distance filter — look up `wrist_link`→`world` transform, extract translation, compute per-point Euclidean distance, create boolean mask for points within `max_distance` (2m).
- [x] **2.5.** Implement hand/arm bbox removal — look up `world`→`wrist_link` transform, apply 4x4 matrix to all xyz points, create boolean mask excluding points within `[bbox_min, bbox_max]` AABB.
- [x] **2.6.** Implement voxel downsampling — numpy-based voxel grid filter (see algorithm below). Average xyz and rgb per voxel.
- [x] **2.7.** Implement PointCloud2 packing — rebuild PointCloud2 from filtered/downsampled numpy arrays with correct header (frame_id="world", timestamp from latest input cloud).
- [x] **2.8.** Implement approximate time synchronizer — use `message_filters.ApproximateTimeSynchronizer` with ~100ms tolerance. Add fallback: if only one cloud arrives, process it alone.
- [x] **2.9.** Implement parameter declaration and loading — all parameters from the Node Design section above, with sensible defaults.
- [x] **2.10.** Implement BBox visualization marker — publish `visualization_msgs/Marker` (CUBE) on `/pointcloud_fusion/hand_removal_bbox` at ~1 Hz for RViz debugging. Frame = `wrist_link`, alpha = 0.3.

#### Voxel Downsampling Implementation (numpy-only)

```python
def voxel_downsample(points_xyz, points_rgb, voxel_size):
    """Voxel grid downsampling using numpy. Returns centroid of each voxel."""
    # Quantize to voxel indices
    voxel_indices = np.floor(points_xyz / voxel_size).astype(np.int32)
    # Create unique voxel keys
    _, unique_indices, inverse = np.unique(
        voxel_indices, axis=0, return_index=True, return_inverse=True
    )
    # Average points per voxel
    n_voxels = len(unique_indices)
    summed_xyz = np.zeros((n_voxels, 3), dtype=np.float64)
    summed_rgb = np.zeros((n_voxels, 3), dtype=np.float64)
    counts = np.zeros(n_voxels, dtype=np.int32)
    np.add.at(summed_xyz, inverse, points_xyz)
    np.add.at(summed_rgb, inverse, points_rgb)
    np.add.at(counts, inverse, 1)
    return summed_xyz / counts[:, None], (summed_rgb / counts[:, None]).astype(np.uint8)
```

This avoids the open3d dependency and is fast enough for ~300K points.

### Step 3: Hand/Arm BBox Dimensions

Based on the Mia hand URDF analysis (`src/mia_hand_description/urdf/mia_hand.urdf.xacro`):

The hand geometry extends roughly:
- **From wrist_link forward along the hand**: ~0.18m (wrist to fingertip)
- **Width across fingers**: ~0.10m
- **Height (palm thickness + fingers)**: ~0.08m

Plus the forearm from wrist to elbow is ~0.25m.

**Recommended AABB in `wrist_link` frame** (with safety margin):

```yaml
bbox_min: [-0.30, -0.10, -0.10]  # 30cm back along forearm, 10cm each side
bbox_max: [0.22, 0.10, 0.12]     # 22cm forward past fingertips, 10cm sides, 12cm up
```

These should be configurable parameters. The x-axis is along the forearm (negative = toward elbow, positive = toward fingertips).

**Note:** The exact axis orientation depends on the `wrist_link` frame convention in the URDF. This needs to be verified at runtime by publishing the bbox as a marker in RViz and adjusting.

### Step 4: Voxel Size Decision

The TSDF resolution in grasp preshaping is **5mm** (`config/grasp_preshaping.yaml:8` — `tsdf_resolution_m: 0.005`).

The voxel downsampling should be **equal to or slightly larger than** the TSDF resolution. Using 5mm voxel downsampling means each voxel cell is the same size as a TSDF voxel — this is the minimum resolution the downstream pipeline can use, so going finer provides no benefit.

**Recommendation:** Start with `voxel_size: 0.005` (5mm). This matches the TSDF resolution exactly. If the point count is still too high for segmentation performance, increase to 0.008 or 0.01. The parameter is configurable at runtime.

### Step 5: Update Launch Configuration

**File:** `src/prosthesis_launch/launch/digital_twin.launch.py`

- [x] **5.1.** Replace the pointcloud merger node (lines 154-168) with the new fusion node from the `pointcloud_fusion` package:

```python
# ── 3. Pointcloud fusion — TF-transforms both clouds to world, merges, filters ──
if camera_enabled:
    nodes.append(
        Node(
            package='pointcloud_fusion',
            executable='pointcloud_fusion_node',
            name='pointcloud_fusion',
            parameters=[{
                'target_frame': 'world',
                'cam1_topic': '/head/d435i_head/depth/color/points',
                'cam2_topic': '/arm/d435i_arm/depth/color/points',
                'arm_frame': 'wrist_link',
                'max_distance': 2.0,
                'voxel_size': 0.005,
                'bbox_min': [-0.30, -0.10, -0.10],
                'bbox_max': [0.22, 0.10, 0.12],
                'enable_downsampling': True,
                'enable_distance_filter': True,
                'enable_hand_removal': True,
            }],
            output='screen',
        )
    )
```

- [x] **5.2.** Change twist propagation's `input_cloud_topic` (line 286) from the raw cam1 topic to `/fused_pointcloud`:

```python
twist_params["input_cloud_topic"] = "/fused_pointcloud"
```

This is critical — currently twist propagation only sees cam1's raw cloud. After this change, it gets the full fused, filtered, downsampled dual-camera cloud.

- [x] **5.3.** Keep the relay node (lines 170-178) unchanged — it bridges `/fused_pointcloud` → `/segmentation/input_cloud` and is still needed.

### Step 6: Update Configuration

**File:** `config/prosthesis_config.yaml`

- [x] **6.1.** Add a new `pointcloud_fusion` section under the ROS2 Node Parameter Sections:

```yaml
pointcloud_fusion:
    ros__parameters:
        target_frame: "world"
        cam1_topic: "/head/d435i_head/depth/color/points"
        cam2_topic: "/arm/d435i_arm/depth/color/points"
        arm_frame: "wrist_link"
        max_distance: 2.0
        voxel_size: 0.005
        bbox_min: [-0.30, -0.10, -0.10]
        bbox_max: [0.22, 0.10, 0.12]
        enable_downsampling: true
        enable_distance_filter: true
        enable_hand_removal: true
```

- [x] **6.2.** Update the twist_propagation section's `input_cloud_topic` from `/camera/depth/color/points` to `/fused_pointcloud`:

```yaml
twist_propagation:
    ros__parameters:
        input_cloud_topic: "/fused_pointcloud"
```

### Step 7: Handle Downstream Compatibility

The current pipeline is:
```
cameras → merger → /fused_pointcloud → relay → /segmentation/input_cloud
```

The new pipeline will be:
```
cameras → fusion_node → /fused_pointcloud → relay → /segmentation/input_cloud
                                            → twist_propagation (direct subscriber)
```

- The relay node (`src/camera/camera/pointcloud_relay_node.py:15-16`) subscribes to `/fused_pointcloud` and publishes to `/segmentation/input_cloud`. This **does not need to change** — the topic names and message types are compatible.
- The segmentation bridge expects xyz+rgb PointCloud2. The new node publishes exactly this.
- Twist propagation currently subscribes to the raw cam1 cloud. After the change, it subscribes to `/fused_pointcloud`, getting the full filtered dual-camera view. Its own internal voxel downsampling (`voxel_leaf_m: 0.02` — 20mm) is much coarser than the fusion node's 5mm, so it will further reduce the cloud for its collision detection use case.

### Step 8: Remove or Deprecate Old Merger

The old `pointcloud_merger_node.py` should be kept in the camera package but removed from the launch file. It can serve as a fallback. Add a comment in the launch file noting it's been replaced by `pointcloud_fusion_node`.

---

## Implementation Order

| Step | Task | File(s) | Est. Effort |
|---|---|---|---|
| 1 | Create package structure (package.xml, setup.py, setup.cfg, resource) | `src/pointcloud_fusion/` (new) | Trivial |
| 2 | Implement fusion node: TF transform + merge only | `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py` (new) | Medium |
| 3 | Add distance filtering (2m from wrist) | Same file | Easy |
| 4 | Add hand/arm bbox removal | Same file | Easy |
| 5 | Add voxel downsampling (5mm) | Same file | Medium |
| 6 | Add bbox visualization marker | Same file | Easy |
| 7 | Update `digital_twin.launch.py` — replace merger, update twist topic | `src/prosthesis_launch/launch/digital_twin.launch.py` | Easy |
| 8 | Add config to `prosthesis_config.yaml` | `config/prosthesis_config.yaml` | Easy |
| 9 | Test with single camera (no TF for cam2) | Runtime | Easy |
| 10 | Test with dual cameras + verify alignment in RViz | Runtime | Medium |
| 11 | Verify segmentation still works with fused/filtered cloud | Runtime | Medium |
| 12 | Verify twist propagation works with fused cloud | Runtime | Medium |
| 13 | Tune bbox dimensions visually via marker | Runtime | Easy |

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| TF not available at startup | High | Clouds dropped until TF ready | Log warnings, fall back to single-cloud mode |
| Voxel downsampling too aggressive (5mm removes too much) | Low | Segmentation misses small objects | Make voxel_size configurable, can increase to 8mm or 10mm |
| BBox too large, removes target object | Medium | Grasp fails | Publish bbox as marker for visual tuning, make configurable |
| BBox axis orientation wrong | Medium | Hand not properly removed | Verify wrist_link frame convention, publish marker |
| Python too slow for dual-cloud processing | Low-Medium | Latency >100ms | Profile; if needed, port critical path to C++ or use numpy vectorization |
| Approximate time sync drops messages | Medium | Reduced update rate | Use 100ms tolerance, fall back to latest-message mode |
| Twist propagation confused by frame change | Low | Collision detection fails | The node parses xyz from any PointCloud2 frame; it uses its own TF2 for pose transforms. Verify frame_id propagation. |

---

## What NOT to Do

1. **Do not add ICP** — The TF chain is accurate enough. ICP would add latency and complexity for marginal benefit. If alignment issues are observed later, ICP can be added as an optional post-processing step.

2. **Do not use Open3D** — Adding a heavy dependency for just voxel downsampling is not justified. The numpy implementation is sufficient and avoids build/deploy complexity, especially on Jetson.

3. **Do not filter before merging** — Filtering in each camera's frame before merging requires looking up the arm position in each camera frame (two extra TF lookups) and is more complex. Filtering after merging in world frame is simpler and only slightly more computationally expensive (operates on more points, but the numpy operations are fast).

4. **Do not remove the old merger or relay** — Keep both as fallback. Just remove the merger from the launch file.

5. **Do not change the relay node** — It works as-is. The fusion node publishes to the same `/fused_pointcloud` topic.

---

## Reference Code from Branches

### From `wt-x86-full-digital-twin` (most relevant):
- `src/camera/camera/pointcloud_merger_node.py` — World-frame fusion approach
- `src/camera/camera/openvins_hand_tracker_node.py` — OpenVINS hand tracking
- `docs/world_frame_fusion_report.md` — Fusion documentation

### From `jetson_docker` (reference):
- `docker_ws/.../src/pointcloud_to_frame_node.cpp` — C++ TF transform + voxel downsampling (reference for algorithm, not for reuse)
- BBox corner transforms — Reference for bbox definition approach

### From current branch (reuse patterns):
- `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:51-77` — PointCloud2 parsing (xyz + rgb extraction)
- `src/twist_propagation/twist_propagation/twist_propagation_node.py:115-127` — Voxel downsampling (first-point-per-voxel approach)
- `src/camera/camera/pointcloud_merger_node.py` — TF2 cloud transformation and byte concatenation pattern
