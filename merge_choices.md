# Merge Choices Documentation

This file documents decisions made during merging two branches into `main_docker`:

1. `rviz_pointcloud_visibility` — Camera/pointcloud fixes and digital twin setup
2. `structural-rework-daniel` — Comprehensive codebase restructuring

---

## Merge 1: `rviz_pointcloud_visibility` → `main_docker` (2026-05-07)

### Conflicts Resolved

| File | Resolution | Rationale |
|------|-----------|-----------|
| `docker_ws/dev/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | Kept rviz version | rviz added `publish_initial_commands_` parameter and changed `preshaping_closure_fraction` from 1.0 → 0.3. These are relevant fixes for the preshaping pipeline. |
| `docker_ws/dev/multiview/Dockerfile.humble_cameras` | Kept rviz version | rviz removed cache mounts (compatibility), added `usbutils`/`v4l-utils`, `rmw-fastrtps-cpp`, and commented out RMW env vars with documentation. Probe script as default CMD. |
| `docker_ws/dev/multiview/launch/multiview_full_launch.py` | Kept rviz version | rviz simplified this to delegate to `two_d435_launch.py`. Removed embedded static TF, charuco, and fusion node (now handled by the delegated launch). |
| `docker_ws/dev/multiview/launch/two_d435_launch.py` | Kept rviz version | rviz improved the camera launch: explicit realsense binary path, `__node` remapping, `__ns` prefix format, `initial_reset:=false`, better static TF using depth optical frames. |
| `docker_ws/dev/multiview/nodes/charuco_tf_node.py` | Kept rviz version | rviz added parameterized camera intrinsics, improved topic paths (`/color/` sub-namespace), removed `os` import, code cleanup. This was an add/add conflict. |
| `docker_ws/docker-deployment/docker-compose.windows.yml` | Combined both | Kept HEAD's `full_system_test` and segmentation services, added rviz's `digital_twin` service. Both sets of functionality are valuable. |

### New Files Added (from rviz)

Files from the `rviz_pointcloud_visibility` branch that did not exist in `main_docker`:

- `docker_ws/dev/multiview/config/realsense_pointcloud.rviz` — RViz config for dual camera pointclouds
- `docker_ws/dev/multiview/Dockerfile.rviz2` — RViz2 visualization container
- `docker_ws/dev/multiview/host_launch_multiview.sh` — Host-side camera launch helper
- `docker_ws/dev/multiview/scripts/realsense_ros_probe.sh` — Camera probe script
- `docker_ws/dev/mujoco/config/digital_twin.rviz` — Digital twin RViz config
- `docker_ws/dev/mujoco/launch/digital_twin_launch.py` — Digital twin orchestration launch
- `docker_ws/dev/mujoco/nodes/pointcloud_relay_node.py` — Pointcloud topic relay

---

## Merge 2: `structural-rework-daniel` → `main_docker` (2026-05-07)

The structural-rework-daniel branch performs a comprehensive restructuring:
- Moves code from `docker_ws/dev/` flat structure to organized `src/` packages
- Creates top-level `config/`, `docker/`, `rviz/`, `scripts/`, `Makefile`
- Removes old `docker_ws/` and `docker_ws/docker-deployment/` in favor of `docker/`
- Adds new nodes: `pipeline_manager`, `force_controller`, `prosthesis_launch`, `wrist_driver`, `twist_propagation`

### Policy: structural-rework is the "main" setup

The structural-rework branch's structure is authoritative. The old `docker_ws/dev/` files that rviz_pointcloud_visibility modified are deleted, and their relevant improvements are ported to the new structure.

### Files Accepted for Deletion (from structural-rework)

All `docker_ws/dev/` files that `rviz_pointcloud_visibility` modified were **deleted** in favor of structural-rework's new `src/` organization:

| Deleted File (old path) | Replacement (new path) | Status |
|------------------------|----------------------|--------|
| `docker_ws/dev/multiview/Dockerfile.humble_cameras` | — | Replaced by `docker/Dockerfile` (Jazzy) |
| `docker_ws/dev/multiview/Dockerfile.rviz2` | — | Not ported — separate container concern |
| `docker_ws/dev/multiview/launch/multiview_full_launch.py` | `src/camera/launch/multiview_full_launch.py` | Ported |
| `docker_ws/dev/multiview/launch/one_d435_launch.py` | `src/camera/launch/single_d435.launch.py` | Already existed in structural |
| `docker_ws/dev/multiview/launch/two_d435_launch.py` | `src/camera/launch/two_d435_launch.py` | Ported |
| `docker_ws/dev/multiview/nodes/charuco_tf_node.py` | `src/camera/camera/charuco_tf_node.py` | Ported as proper ROS package node |
| `docker_ws/dev/multiview/config/realsense_pointcloud.rviz` | `rviz/realsense_pointcloud.rviz` | Ported to top-level rviz/ |
| `docker_ws/dev/multiview/scripts/realsense_ros_probe.sh` | `scripts/realsense_ros_probe.sh` | Ported |
| `docker_ws/dev/multiview/host_launch_multiview.sh` | `scripts/host_launch_multiview.sh` | Ported |
| `docker_ws/dev/mujoco/config/digital_twin.rviz` | — | Not ported — mujoco removed in structural |
| `docker_ws/dev/mujoco/launch/digital_twin_launch.py` | — | Not ported — references old paths |
| `docker_ws/dev/mujoco/nodes/pointcloud_relay_node.py` | — | Not ported — simple relay, can be recreated |
| `docker_ws/dev/grasp_preshaping/files` | `src/grasp_preshaping/` | Renamed via git (auto-merge) |
| `docker_ws/docker-deployment/` | `docker/` | Replaced by new structure |
| `multiview/` (root-level) | — | Deleted — was a duplicate of docker_ws/dev/multiview/ |

### Content Conflicts Resolved

| File | Resolution | Rationale |
|------|-----------|-----------|
| `.gitignore` | Combined both | Kept `.agents/` from rviz branch and `!libgrasp_preshaping.so` from structural. Both entries serve different purposes. |
| `README.md` | Kept structural-rework version | Structural-rework has a clean, reorganized README documenting the new `src/` structure. The old README's path references are outdated. Camera setup docs will need to be ported separately. |

### Auto-Merged Files Verified

| File | Status | Notes |
|------|--------|-------|
| `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp` | ✓ Good | rviz changes (`publish_initial_commands_`, `preshaping_closure_fraction=0.3`) properly merged into new location |
| `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py` | ✓ Good | Structural-rework version kept (new proximity controller) |

### Files Ported to New Structure

The following rviz_pointcloud_visibility improvements were manually ported to the structural-rework structure:

| New File | Source | Purpose |
|----------|--------|---------|
| `src/camera/launch/two_d435_launch.py` | rviz `docker_ws/dev/multiview/launch/two_d435_launch.py` | Dual D435 camera launch with serial YAML quoting workaround |
| `src/camera/launch/multiview_full_launch.py` | rviz `docker_ws/dev/multiview/launch/multiview_full_launch.py` | Simplified delegator to `two_d435_launch.py` |
| `src/camera/camera/charuco_tf_node.py` | rviz `docker_ws/dev/multiview/nodes/charuco_tf_node.py` | Improved ChArUco board detector with parameterized intrinsics |
| `rviz/realsense_pointcloud.rviz` | rviz `docker_ws/dev/multiview/config/realsense_pointcloud.rviz` | RViz config for dual camera pointcloud with charuco board frame |
| `scripts/realsense_ros_probe.sh` | rviz `docker_ws/dev/multiview/scripts/realsense_ros_probe.sh` | Camera probe and launch script |
| `scripts/host_launch_multiview.sh` | rviz `docker_ws/dev/multiview/host_launch_multiview.sh` | Host-side RealSense launch helper |
| `src/camera/camera/__init__.py` | New | Package init for Python import |
| `src/camera/setup.py` (updated) | New | Added charuco_tf_node entry point, opencv/cv-bridge deps |

### Files NOT Ported (with reasons)

| File | Reason Not Ported |
|------|------------------|
| `docker_ws/dev/multiview/Dockerfile.rviz2` | Structural-rework uses Jazzy; rviz2 Dockerfile used Humble. Separate container deployment is a deployment concern. Can be recreated if RViz2 is needed in a separate container. |
| `docker_ws/dev/multiview/Dockerfile.humble_cameras` | Structural-rework uses `docker/Dockerfile` for Jazzy-based ROS. Camera config is now part of that. |
| `docker_ws/dev/mujoco/config/digital_twin.rviz` | mujoco directory was removed in structural-rework. Digital twin concept needs rethinking for the new structure. |
| `docker_ws/dev/mujoco/launch/digital_twin_launch.py` | References old paths (`/miahand_ws/src/dev/...`). Needs adaptation to new structure. |
| `docker_ws/dev/mujoco/nodes/pointcloud_relay_node.py` | Simple relay node (sub→pub). Easy to recreate if needed. |
| `multiview/` (root-level files) | These were duplicates/copies of files in `docker_ws/dev/multiview/`. |
| `docker_ws/dev/grasp_preshaping/README.md` changes | rviz removed backside shape estimation docs. Structural-rework has its own README. |
| `.agents/` planning documents | These were implementation plans, not code. Not relevant for the final merge. |
