# Full System Integration Status

## Current State

The full digital-twin pipeline is wired to run on this x86 machine with the
final two-D435i setup:

- Local RealSense containers publish final camera topics and OpenVINS-compatible
  outputs.
- The digital twin fuses both pointclouds in `world`.
- Collision prediction publishes segmentation clicks.
- Segmentation output feeds grasp preshaping.
- Grasp command topics drive the RViz hand model through `/joint_states`.
- Physical hand hardware and force grasping are bypassed.

## Main Commands

Start full system:

```bash
make up-full-digital-twin
```

Stop full system:

```bash
make stop-full-digital-twin
```

Run full smoke suite:

```bash
make test
```

Run live topic check after startup:

```bash
make check-full-pipeline
```

## Verified Tests

`make test` passes in the host test container:

- build
- launch
- preshaping
- nodes_start
- twist_prop
- ros_network
- digital_twin
- hardware_bypass
- full_pipeline
- segmentation
- grasp
- final_camera
- final_openvins
- mixed_camera

Result: 14 passed, 0 failed.

## Runtime Status

The default `make up-full-digital-twin` path no longer requires Jetson SSH or
Ethernet. It expects the two D435i cameras to be connected locally and visible
through `/dev` inside the `x86_cameras` container.

Legacy Jetson perception is still available explicitly:

```bash
DT_PERCEPTION_BACKEND=jetson make up-full-digital-twin
STOP_JETSON=1 make stop-full-digital-twin
```

## Rollback Points

Recent integration commits are intentionally scoped:

- final dual-D435i defaults
- final OpenVINS wiring
- ROS network alignment
- world-frame pointcloud fusion
- cloud timestamp TF lookup
- segmentation and grasp contracts
- digital-twin hardware bypass
- full pipeline orchestration
- smoke-suite stabilization
