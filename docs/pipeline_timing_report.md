# Pipeline Timing Report

## Scope

Timing-sensitive fusion now uses pointcloud message stamps for TF lookup.

## Change

`pointcloud_merger_node` previously requested the latest available transform for
cloud fusion. That can fuse a cloud with a transform from a different time,
especially when OpenVINS and camera topics cross machines.

The merger now:

- uses `cloud.header.stamp` for TF lookup by default
- falls back to latest TF only for zero-stamped messages
- exposes `use_cloud_timestamps`
- exposes `tf_timeout_s`

## Runtime Check

After Jetson and host pipeline are running:

```bash
bash scripts/check_pipeline_timing_topics.sh
```

This checks key pipeline outputs carry stamped headers:

- `/fused_pointcloud`
- `/segmentation/input_cloud`
- `/hand_pose`
- `/hand_twist`

## Limit

Live timing validation still requires the Jetson ethernet link to have carrier
and the final camera/OpenVINS topics to be visible on the host.
