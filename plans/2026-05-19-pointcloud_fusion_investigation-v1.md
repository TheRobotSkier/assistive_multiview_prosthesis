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
| **Distance filtering?** | **Yes** | Remove points >3m from wrist. Reduces cloud size significantly (most of a 4m D435i range is irrelevant). Do **after** merging in world frame for simplicity. |
| **Hand/arm removal?** | **Yes** | Use AABB in arm frame (wrist_link). The jetson_docker branch has bbox corner transforms. The wt-x86 branch documents this approach. Define bbox in `wrist_link` frame, transform to world frame, remove interior points. |
| **Target frame?** | **`world`** | Both branches converge on world-frame fusion. The current branch uses `cam1_depth_optical_frame` which is wrong for proper dual-camera fusion — it biases toward cam1 and doesn't properly handle the case where cam2 has a better view. World frame is frame-agnostic. |
| **Extend or replace merger?** | **Replace** | The current `pointcloud_merger_node.py` transforms cam2→cam1 frame (wrong approach for proper fusion). The new node should transform **both** to world frame. The existing node can be kept as fallback but the new fusion node should be a separate, clean implementation. |

---

## Implementation Plan

### Step 1: Create `pointcloud_fusion_node.py`

**File:** `src/camera/camera/pointcloud_fusion_node.py`

This is the main new node. It replaces the current merger + relay pipeline.

#### Architecture

```
/cam1/depth/color/points  ──┐
                              ├──> [pointcloud_fusion_node] ──> /fused_pointcloud
/cam2/depth/color/points  ──┘          │
                                        ├── TF2: cam*_optical_frame → world
                                        ├── Merge both clouds in world frame
                                        ├── Distance filter (3m from wrist_link)
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
    cam1_topic: str                       # e.g., /cam1/depth/color/points
    cam2_topic: str                       # e.g., /cam2/depth/color/points
    arm_frame: str = "wrist_link"         # Frame for distance filtering & bbox
    max_distance: float = 3.0             # Max distance from arm (meters)
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

4. **Distance filter (3m from arm)** — Look up `wrist_link` position in world frame via TF2. Compute Euclidean distance from each point to wrist position. Remove points beyond `max_distance`. This is a simple numpy operation on the xyz data.

5. **Hand/arm bbox removal** — Define an AABB in `wrist_link` frame. Transform all remaining points into `wrist_link` frame (one matrix multiply using the TF). Remove points where all three axes fall within the bbox bounds. Transform is done once for the whole array.

6. **Voxel downsampling** — Apply voxel grid filter. For each voxel cell, keep the centroid point (average xyz and rgb). This reduces ~200K-400K points to ~50K-80K depending on voxel size. Use numpy-based implementation (no open3d dependency needed).

7. **Publish** — Pack the filtered, downsampled points back into a `PointCloud2` message with `frame_id = "world"` and publish on `/fused_pointcloud`.

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

### Step 2: Hand/Arm BBox Dimensions

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

### Step 3: BBox Visualization

To aid debugging, the node should publish the hand removal bbox as a `visualization_msgs/Marker` (CUBE type) in RViz. This allows visual verification that the bbox correctly covers the hand/arm.

- Publish on `/hand_removal_bbox` topic
- Use `wrist_link` as frame_id
- Set alpha to 0.3 for transparency
- Update at ~1 Hz (no need for every cloud)

### Step 4: Update Launch Configuration

**File:** `src/prosthesis_launch/launch/digital_twin.launch.py`

Replace the current merger node launch with the new fusion node:

```python
# Replace:
# Node(package='camera', executable='pointcloud_merger_node', ...)

# With:
Node(
    package='camera',
    executable='pointcloud_fusion_node',
    parameters=[{
        'target_frame': 'world',
        'cam1_topic': '/cam1/depth/color/points',
        'cam2_topic': '/cam2/depth/color/points',
        'arm_frame': 'wrist_link',
        'max_distance': 3.0,
        'voxel_size': 0.005,
        'bbox_min': [-0.30, -0.10, -0.10],
        'bbox_max': [0.22, 0.10, 0.12],
        'enable_downsampling': True,
        'enable_distance_filter': True,
        'enable_hand_removal': True,
    }],
    remappings=[],
    output='screen',
)
```

**File:** `config/prosthesis_config.yaml`

Add a new section:

```yaml
pointcloud_fusion:
  target_frame: "world"
  arm_frame: "wrist_link"
  max_distance: 3.0
  voxel_size: 0.005
  bbox_min: [-0.30, -0.10, -0.10]
  bbox_max: [0.22, 0.10, 0.12]
  enable_downsampling: true
  enable_distance_filter: true
  enable_hand_removal: true
```

### Step 5: Update `setup.py` Entry Point

**File:** `src/camera/setup.py`

Add the new entry point:

```python
'pointcloud_fusion_node = camera.pointcloud_fusion_node:main',
```

### Step 6: Handle Downstream Compatibility

The current pipeline is:
```
cameras → merger → /fused_pointcloud → relay → /segmentation/input_cloud
```

The new pipeline will be:
```
cameras → fusion_node → /fused_pointcloud → relay → /segmentation/input_cloud
```

The relay node (`src/camera/camera/pointcloud_relay_node.py`) subscribes to `/fused_pointcloud` and publishes to `/segmentation/input_cloud`. This **does not need to change** — the topic names and message types are compatible.

The segmentation bridge expects xyz+rgb PointCloud2 in `world` frame. The new node publishes exactly this.

### Step 7: Remove or Deprecate Old Merger

The old `pointcloud_merger_node.py` should be kept but removed from the launch file. It can serve as a fallback if the new fusion node has issues. Add a comment in the launch file noting it's been replaced.

---

## Implementation Order

| Step | Task | File(s) | Est. Effort |
|---|---|---|---|
| 1 | Create fusion node with TF transform + merge only | `src/camera/camera/pointcloud_fusion_node.py` (new) | Medium |
| 2 | Add distance filtering | Same file | Easy |
| 3 | Add hand/arm bbox removal | Same file | Easy |
| 4 | Add voxel downsampling | Same file | Medium |
| 5 | Add bbox visualization marker | Same file | Easy |
| 6 | Update setup.py entry point | `src/camera/setup.py` | Trivial |
| 7 | Update launch file | `src/prosthesis_launch/launch/digital_twin.launch.py` | Easy |
| 8 | Add config parameters | `config/prosthesis_config.yaml` | Easy |
| 9 | Test with single camera (no TF for cam2) | Runtime | Easy |
| 10 | Test with dual cameras + verify alignment | Runtime | Medium |
| 11 | Verify segmentation still works | Runtime | Medium |
| 12 | Tune bbox dimensions visually | Runtime | Easy |

---

## Risks and Mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| TF not available at startup | High | Clouds dropped until TF ready | Log warnings, fall back to single-cloud mode |
| Voxel downsampling too aggressive | Medium | Segmentation misses small objects | Make voxel_size configurable, start with 5mm |
| BBox too large, removes target object | Medium | Grasp fails | Publish bbox as marker for visual tuning, make configurable |
| BBox axis orientation wrong | Medium | Hand not properly removed | Verify wrist_link frame convention, publish marker |
| Python too slow for dual-cloud processing | Low-Medium | Latency >100ms | Profile; if needed, port critical path to C++ or use numpy vectorization |
| Approximate time sync drops messages | Medium | Reduced update rate | Use 100ms tolerance, fall back to latest-message mode |

---

## What NOT to Do

1. **Do not add ICP** — The TF chain is accurate enough. ICP would add latency and complexity for marginal benefit. If alignment issues are observed later, ICP can be added as an optional post-processing step.

2. **Do not use Open3D** — Adding a heavy dependency for just voxel downsampling is not justified. The numpy implementation is sufficient and avoids build/deploy complexity, especially on Jetson.

3. **Do not filter before merging** — Filtering in each camera's frame before merging requires looking up the arm position in each camera frame (two extra TF lookups) and is more complex. Filtering after merging in world frame is simpler and only slightly more computationally expensive (operates on more points, but the numpy operations are fast).

4. **Do not remove the old merger** — Keep it as fallback. Just remove it from the launch file.

---

## Reference Code from Branches

### From `wt-x86-full-digital-twin` (most relevant):
- `src/camera/camera/pointcloud_merger_node.py` — World-frame fusion approach
- `src/camera/camera/openvins_hand_tracker_node.py` — OpenVINS hand tracking
- `docs/world_frame_fusion_report.md` — Fusion documentation

### From `jetson_docker` (reference):
- `docker_ws/.../src/pointcloud_to_frame_node.cpp` — C++ TF transform + voxel downsampling (reference for algorithm, not for reuse)
- BBox corner transforms — Reference for bbox definition approach
