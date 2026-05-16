# Collision Click Loop Report

## Scope

Verified the collision prediction to positive-click path used by automated
segmentation.

## Fix

The integration test runner did not start `twist_propagation_node`, so the test
only waited for services that could never appear. The runner now launches the
node, runs the harness, and cleans the node up on exit.

## Interface

Inputs:

- `/hand_pose`
- `/fused_pointcloud` in the digital twin launch
- `/segmentation/object_cloud`

Outputs:

- `/hand_twist`
- `/segmentation/click_positive`
- `/twist_propagation/status`

On hit, the click is published in the cloud frame, so
`segmentation_ros2_node` can consume it directly or transform it if needed.

## Test

Passed in container:

```bash
cd docker
podman-compose --profile test run --rm test bash /prosthesis_ws/scripts/test_twist_propagation.sh
```

Result: 11 passed, 0 failed.
