# Full Pipeline Orchestration Report

## Goal

Run the final dual-D435i Jetson perception stack and the host digital-twin grasp
stack with one command, while keeping physical hand hardware disabled.

## Commands

Start everything:

```bash
make up-full-digital-twin
```

This syncs committed code to the Jetson, starts final camera and OpenVINS
containers there, then starts the host segmentation and digital-twin containers.

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

Jetson publishes final camera and OpenVINS topics:

- `/head/d435i_head/depth/color/points`
- `/arm/d435i_arm/depth/color/points`
- `/ov_msckf_head/odomimu`
- `/ov_msckf_arm/odomimu`

Host digital twin consumes those topics, fuses clouds into `/fused_pointcloud`,
runs segmentation and grasp planning, and renders the hand through
`/joint_states`. No serial devices or hardware-control profile are used.

## Notes

`SKIP_JETSON=1 make up-full-digital-twin` starts only the host side when the
Jetson stack is already running. `STOP_JETSON=0 make stop-full-digital-twin`
leaves the Jetson containers running.
