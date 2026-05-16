# Mixed Camera Integration Plan

Date: 2026-05-16

## Context

Current hardware is one D435i and one D435 without onboard IMU. The D435 has an
external GY-91-style IMU on the Jetson I2C connector. This is a temporary bench
configuration until the second D435i arrives, so all mixed-camera behavior must
be reversible through launch/config selection.

## Sprint 2: Mixed-Camera OpenVINS Bringup

Goal: make the Jetson camera/OpenVINS side usable with D435 + external I2C IMU.

Tasks:

- Discover live camera serials from the Jetson.
- Add a reversible camera rig mode:
  - `dual_d435i` for the final two-D435i setup.
  - `mixed_d435_d435i` for the current D435 + D435i bench setup.
- Ensure the D435 RealSense node does not request gyro/accel streams.
- Ensure the external IMU publisher is launched for the D435 OpenVINS estimator.
- Keep all serials and calibration paths in small YAML files.
- Add a smoke test that validates the mixed-mode launch/config contract.

Commit when the launch/config contract builds and tests pass.

## Sprint 3: Jetson Runtime Smoke Tests

Goal: verify live ROS topics from the Jetson before running the full host stack.

Tasks:

- Sync sprint 2 to the Jetson.
- Start mixed-mode camera containers.
- Verify pointcloud/image topics for both cameras.
- Verify the external IMU topic for the D435.
- Verify ArUco ID 1 is detected from both camera image streams.
- Start OpenVINS mixed mode and inspect odometry/TF/marker observation topics.

Commit any runtime fixes after each working milestone.

## Sprint 4: Host Full Pipeline With Mixed Cameras

Goal: run the integrated pipeline with physical pointclouds and RViz hand render.

Tasks:

- Start Jetson mixed OpenVINS/camera stack.
- Start host digital twin pipeline.
- Verify fused pointcloud appears on `/fused_pointcloud`.
- Verify collision prediction publishes `/segmentation/click_positive`.
- Verify segmentation produces `/segmentation/object_cloud`.
- Verify preshaping publishes planner outputs.
- Verify RViz hand closes from planner/controller output.

Commit after the host path can run end-to-end or after any isolated fix that
passes its own test.

## Sprint 5: Revert Switch For Final D435i Pair

Goal: leave the repo ready for the real two-D435i hardware.

Tasks:

- Document the exact Makefile/config switch from mixed mode to final dual-D435i.
- Keep mixed mode available as a fallback without editing source code.
- Add a checklist for the first dual-D435i OpenVINS run.

Commit docs and any final Makefile cleanup.
