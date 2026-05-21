# Camera Mount Visualization

Offline RViz2 visualization of D435i camera mounts on the Mia hand prosthesis, with workspace bounding box.

## Overview

Visualizes the spatial relationship between the D435i camera body and the Mia hand palm frame across different mounting configurations. Uses the D435i's tripod mount (`bottom_screw_frame`) as the common reference between the URDF model and the CAD assembly.

## Files

```
src/sensor_fusion_bringup/
  config/camera_mounts.yaml          — Mount transforms + bounding box definition
  scripts/publish_camera_mounts.py   — ROS2 node: reads YAML, publishes TF + markers

rviz/camera_mounts.rviz              — RViz2 config (TF axes, bounding box marker)
Makefile.rviz                        — Docker-based launch targets
```

## TF Tree

```
world ── palm_frame                           (identity)
  │
  ├── d435i_arm_bottom_screw_frame_5_cm_cam_mount
  │     └── d435i_arm_link_5_cm_cam_mount
  │
  ├── d435i_arm_bottom_screw_frame_8_cm_cam_mount
  │     └── d435i_arm_link_8_cm_cam_mount
  │
  ├── ... (all 5 mounts)
  │
  └── bb_corner ── bb_opposite                 (workspace bounding box)
```

- **palm_frame**: Mia hand base frame (anchor point)
- **d435i_arm_bottom_screw_frame_<mount>**: D435i tripod mount hole center (X=forward/lens, Y=left, Z=up)
- **d435i_arm_link_<mount>**: D435i camera body origin (URDF offset: +10.6, +17.5, +12.5 mm)
- **bb_corner / bb_opposite**: Workspace bounding box corners in palm frame

## Mounts

| Name | Description | Camera Position (from screw frame) |
|------|-------------|-----------------------------------|
| 5_cm_cam_mount | 5 cm above hand, vertical | Z = 130.955 mm |
| 8_cm_cam_mount | 8 cm above hand, vertical | Z = 160.955 mm |
| 10_cm_cam_mount | 10 cm above hand, vertical | Z = 180.955 mm |
| 12_cm_V_cam_mount | 12 cm above hand, vertical | Z = 200.955 mm |
| 12_cm_H_cam_mount | 12 cm away, horizontal | Y = -162.955 mm, Z = -53.762 mm |

Mounts 1-4 share the same orientation: `palm x = screw -z, palm y = screw x, palm z = screw -y`.  
Mount 5 (horizontal) uses: `palm x = screw y, palm y = screw x, palm z = screw -z`.

## Usage

### Prerequisites

```bash
# Ensure Docker is installed and X11 forwarding works
xhost +local:
```

### Visualize All Mounts (default)

```bash
make -f Makefile.rviz mounts-viz
```

RViz2 opens showing:
- All 5 camera mount positions simultaneously
- TF axes (colored) + frame names for each mount
- Green semi-transparent workspace bounding box (CUBE marker)

### Visualize Single Mount

```bash
make -f Makefile.rviz mounts-viz MOUNT=5_cm_cam_mount
make -f Makefile.rviz mounts-viz MOUNT=12_cm_H_cam_mount
```

### Stop

```bash
make -f Makefile.rviz mounts-viz-kill
```

### CLI Only (no GUI)

```bash
# List available mounts
python3 src/sensor_fusion_bringup/scripts/publish_camera_mounts.py --list

# Publish TFs (all mounts)
python3 src/sensor_fusion_bringup/scripts/publish_camera_mounts.py --all

# Publish TFs (single mount)
python3 src/sensor_fusion_bringup/scripts/publish_camera_mounts.py --mount 5_cm_cam_mount
```

## Frame Conventions

### bottom_screw_frame (D435i)
- **Origin**: Center of the ¼"-20 tripod mount hole on the bottom face
- **X**: Forward (lens pointing direction)
- **Y**: Left (when looking from rear; stereo baseline along Y)
- **Z**: Up

### palm_frame (Mia Hand)
- **Origin**: Mia hand base
- Orientation defined in `camera_mounts.yaml` per mount

### Bounding Box (Workspace)
```
palm_frame → bb_corner      : (+0.110, +0.205, +0.110) m
bb_corner  → bb_opposite    : (-0.220, -0.340, -0.320) m
```
Box centered at (0, 0.035, -0.05) m in palm frame, dimensions 0.22 × 0.34 × 0.32 m.

## Adding a New Mount

1. Measure the transform `bottom_screw_frame → palm_frame` in your CAD software (mm)
2. Determine the orientation: how palm frame axes map to bottom_screw_frame axes
3. Add an entry to `camera_mounts.yaml` under `mounts:` following the existing format
4. Ensure translations are in **meters** and the quaternion is a unit quaternion (norm = 1.0)

## How It Works

1. `camera_mounts.yaml` stores all transforms in meters
2. `publish_camera_mounts.py` reads the YAML and:
   - Publishes `world → palm_frame` (identity)
   - For each mount, computes the **inverse** of `screw → palm` to get `palm → screw_frame`
   - Appends the URDF offset `screw_frame → camera_link` (+10.6, +17.5, +12.5 mm)
   - Publishes the workspace bounding box as TF frames + a CUBE marker at 1 Hz
3. RViz2 loads `camera_mounts.rviz` which shows:
   - TF display (all frames, colored axes, names)
   - Bounding box marker (semi-transparent green CUBE)
