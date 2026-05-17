# Full Pipeline Orchestration Report

## Goal

Run the final dual-D435i perception stack and the digital-twin grasp stack on
one x86 machine with one command, while keeping physical hand hardware disabled.

## Commands

Start everything:

```bash
make up-full-digital-twin
```

This starts local camera, OpenVINS-compatible marker odometry, segmentation,
digital-twin, and RViz containers. The RealSense cameras must be connected to
this x86 machine.

Stop everything:

```bash
make stop-full-digital-twin
```

Runtime check:

```bash
make check-full-pipeline
```

Static contract check:

```bash
bash scripts/check_full_pipeline_contracts.sh
```

## Interfaces

The x86 perception containers publish the same final camera and
OpenVINS-compatible topics used by the Jetson setup:

- `/head/d435i_head/depth/color/points`
- `/arm/d435i_arm/depth/color/points`
- `/ov_msckf_head/odomimu`
- `/ov_msckf_arm/odomimu`

The digital twin consumes those topics, fuses clouds into `/fused_pointcloud`,
runs segmentation and grasp planning, and renders the hand through
`/joint_states`. No serial devices or hardware-control profile are used.

## Notes

`DT_PERCEPTION_BACKEND=jetson make up-full-digital-twin` keeps the legacy
Jetson perception path available for comparison.
`DT_PERCEPTION_BACKEND=mock make up-full-digital-twin` starts the digital-twin
stack without physical cameras.

If the legacy Jetson backend was started, use
`STOP_JETSON=1 make stop-full-digital-twin` to stop the Jetson containers too.
