# Full System Integration Status

## Current State

The host-side digital-twin pipeline is wired for the final two-D435i Jetson
setup:

- Jetson publishes final camera topics and OpenVINS outputs.
- Host fuses both pointclouds in `world`.
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

## Jetson Status

Deploy/sync is currently blocked before SSH because the direct Ethernet adapter
has no physical carrier:

```text
ERROR: Ethernet adapter enp0s13f0u2u2 has no carrier.
Host IP is configured as 192.168.100.1, but no physical link is detected.
```

Once the link is up, run:

```bash
make robotlab-connect
make up-full-digital-twin
make check-full-pipeline
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
