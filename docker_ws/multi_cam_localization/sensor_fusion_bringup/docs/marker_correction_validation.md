# Marker Correction Validation

Run these inside the ROS 2 Jazzy container after building and sourcing the workspace.

## Live Head D435i

```bash
ros2 launch sensor_fusion_bringup head_d435i_openvins.launch.py
ros2 launch sensor_fusion_bringup launch/head_marker_pose.launch.py
```

Watch the correction status:

```bash
ros2 topic echo /head/marker_pose/marker_valid
ros2 topic echo /head/marker_pose/vio_valid
ros2 topic echo /head/marker_pose/reanchor_event
```

Pose topics:

- `/head/marker_pose/camera_pose_raw`: raw ROS/OpenCV camera optical frame. Optical axes are `+X` right, `+Y` down, `+Z` forward through the lens.
- `/head/marker_pose/camera_body_pose`: visualization-only body frame in `marker_map`. Axes are `+X` forward, `+Y` left, `+Z` up. This topic is not used by the correction filter.
- `/head/marker_pose/imu_pose`: true calibrated IMU pose in `marker_map`. Its axes are the actual calibrated IMU frame and may not be intuitive in RViz.
- `/head/marker_pose/ov_corrected_odom`: corrected OpenVINS odom in `marker_map`.

Request a manual hard reanchor:

```bash
ros2 service call /head/marker_pose/request_reanchor std_srvs/srv/Trigger {}
```

## RViz Frame Direction Check

1. Open RViz with Fixed Frame set to `marker_map`.
2. Keep marker ID 0 visible until `/head/marker_pose/marker_valid` is true and an initial lock event is published.
3. Do not judge camera viewing direction from the RViz pose arrow on `/head/marker_pose/camera_pose_raw`; RViz arrows point along `+X`, while camera optical viewing direction is blue `+Z`.
4. Confirm the blue `+Z` axis of `/head/marker_pose/camera_pose_raw` points through the lens toward the marker when the camera looks at the marker.
5. Use `/head/marker_pose/camera_body_pose` when you want an intuitive display frame where the red `+X` axis points forward through the lens.
6. Move the camera physically upward while the marker remains visible and confirm numeric `/head/marker_pose/imu_pose` position z increases in `marker_map`.
7. Move the camera physically right and confirm one horizontal axis changes consistently.
8. Move the camera closer/farther from the marker and confirm the depth axis changes, not `marker_map` z.
9. Yaw the camera and confirm the motion looks like yaw around `marker_map` z, not pitch.

The current marker 0 default applies the empirical remap:

```text
new_x = old_x
new_y = old_z
new_z = -old_y
```

Do not change `imu_pose` axes just to make RViz look nicer. Display-only axis preferences belong in `/head/marker_pose/camera_body_pose`.

## Recorded Bag Replay

For detailed commands, see:

```text
multi_cam_localization/sensor_fusion_bringup/docs/head_openvins_marker_bag_replay_validation.md
```

A good Phase 1 replay-validation bag should include:

```text
/head/d435i_head/color/image_raw
/head/d435i_head/color/camera_info
/head/d435i_head/imu
/ov_msckf/odomimu
/ov_msckf/poseimu
/ov_msckf/pathimu
/head/marker_pose/*
/tf
/tf_static
/rosout
```

Scenarios verified:

- Marker visible while OpenVINS/VIO is healthy.
- Marker lost and regained while OpenVINS/VIO is healthy.
- VIO drift case: camera moves to a texture-only view, OpenVINS drifts, then marker is seen again.
- Manual reanchor service accepts when marker quality and odom timestamp matching pass.
- `reanchor_event` contains accepted/rejected reasons.
- Corrected odom reanchors in `marker_map` when marker is visible again.
- Replay option A: recorded marker outputs inspected in RViz2.
- Replay option B: marker outputs regenerated from raw image + OpenVINS odom replay.

## Completed Phase 1 Replay Validation

Detailed command recipes and full bag-info notes are in:

```text
multi_cam_localization/sensor_fusion_bringup/docs/head_openvins_marker_bag_replay_validation.md
```

Completed:

- D435i + OpenVINS live launch.
- Marker pose node live launch.
- RViz2 live view using `head_openvins.rviz`.
- Manual reanchor test, including during recording.
- Compact validation bag recorded.
- Full validation bags recorded.
- Recorded-output replay inspected in RViz2.
- Raw-input replay used to regenerate marker outputs.
- Regenerated marker topics inspected.

Motion sequence used:

1. OpenVINS initialized and marker was visible while VIO was good.
2. Camera moved away from marker while VIO was still good.
3. Camera returned to marker while VIO was still good.
4. Sequence repeated a few times.
5. Camera moved away to texture-only view where VIO began drifting.
6. Camera turned back to marker while VIO was bad.
7. Manual reanchor was sent.
8. Recording was stopped.

Observed behavior:

- During good-VIO segments, marker-corrected odom behaved as expected.
- During bad-VIO/drifting segment, corrected odom snapped/reanchored back toward the right marker-map pose when marker was visible.
- Between marker updates, output drifted because raw OpenVINS was still drifting.
- Pose covariance continued growing in the bad-VIO case.
- This is expected for Phase 1 external correction because it does not update the internal OpenVINS EKF state, velocity, or covariance.
- This observation motivates future Phase 2 work: internal OpenVINS marker update/reanchor that can correct pose, velocity, and covariance.

The bad-VIO behavior is an important Phase 1 boundary: the external corrected
odom can reanchor the published output when marker observations return, but it
does not repair OpenVINS' internal state. Between marker corrections,
`/head/marker_pose/ov_corrected_odom` still inherits drift from
`/ov_msckf/odomimu`.

## Actual validation bags recorded on 2026-05-06

Newest validation bags found:

### `head_marker_phase1_full_validation_20260506_124409`

```text
Bag size: 1.8 GiB
Duration: 71.218826315 s
Start: May  6 2026 12:44:10.386254153
End:   May  6 2026 12:45:21.605080468
Messages: 88992
```

Recorded topics included:

```text
/head/d435i_head/color/camera_info
/head/d435i_head/color/image_raw
/head/d435i_head/imu
/head/marker_pose/active_marker_id
/head/marker_pose/camera_body_pose
/head/marker_pose/camera_pose_raw
/head/marker_pose/imu_pose
/head/marker_pose/marker_valid
/head/marker_pose/ov_corrected_odom
/head/marker_pose/reanchor_event
/head/marker_pose/vio_valid
/ov_msckf/loop_extrinsic
/ov_msckf/loop_feats
/ov_msckf/loop_intrinsics
/ov_msckf/loop_pose
/ov_msckf/odomimu
/ov_msckf/pathimu
/ov_msckf/poseimu
/rosout
/tf
/tf_static
```

Important counts:

```text
/head/d435i_head/color/image_raw: 1903
/head/d435i_head/color/camera_info: 2134
/head/d435i_head/imu: 14073
/ov_msckf/odomimu: 8609
/ov_msckf/poseimu: 1742
/ov_msckf/pathimu: 1742
/head/marker_pose/imu_pose: 654
/head/marker_pose/ov_corrected_odom: 8623
/head/marker_pose/reanchor_event: 105
/tf: 29418
/tf_static: 1
```

### `head_marker_phase1_full_validation_20260506_122604`

```text
Bag size: 2.0 GiB
Duration: 79.842433792 s
Start: May  6 2026 12:26:05.301420442
End:   May  6 2026 12:27:25.143854234
Messages: 91200
```

Recorded topics included the same Phase 1 full-validation topic set.

Important counts:

```text
/head/d435i_head/color/image_raw: 2130
/head/d435i_head/color/camera_info: 2393
/head/d435i_head/imu: 15768
/ov_msckf/odomimu: 8016
/ov_msckf/poseimu: 1973
/ov_msckf/pathimu: 1973
/head/marker_pose/imu_pose: 868
/head/marker_pose/ov_corrected_odom: 7937
/head/marker_pose/reanchor_event: 171
/tf: 28695
/tf_static: 1
```
