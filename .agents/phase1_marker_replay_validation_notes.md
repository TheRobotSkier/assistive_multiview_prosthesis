# Phase 1 Marker-Corrected Odom Replay Validation Notes

## Validation summary

A replay validation was completed for the head D435i OpenVINS + external marker-corrected odom stack.

Completed actions:

- Started live D435i + OpenVINS.
- Started marker pose node.
- Opened RViz2 with `head_openvins.rviz` for live recording.
- Ran manual reanchor test, also while recording.
- Recorded compact validation bag.
- Recorded full validation bag.
- Replayed recorded-output bag.
- Opened RViz2 during recorded-output replay.
- Inspected replayed marker topics.
- Started raw replay container.
- Started marker node during raw replay.
- Opened RViz2 during raw-input replay.
- Inspected regenerated marker outputs.

## Recorded sequence

The recording was intentionally made to include both healthy and bad VIO:

1. OpenVINS initialized successfully.
2. Marker was visible while VIO was good.
3. Camera moved away from the marker while VIO was good.
4. Camera returned to the marker while VIO was good.
5. This may have repeated a few times.
6. Camera moved away to a texture-only view where VIO began drifting.
7. Camera turned back to the marker while VIO was bad.
8. Manual reanchor was sent.
9. Recording was stopped.

## Observed behavior

- When VIO was good and marker was visible, corrected pose behaved as expected.
- When VIO was bad/drifting and marker became visible, corrected odom snapped/reanchored back toward the right marker-map pose.
- Between marker corrections, corrected odom continued to drift because raw OpenVINS odom was still drifting.
- Pose covariance continued growing in the bad-VIO case.
- This is expected for Phase 1 external correction because Phase 1 does not modify the OpenVINS internal EKF state, velocity, or covariance.

## Interpretation

Phase 1 is useful as an external corrected output and as a robust marker-map reference, but it cannot fully stop OpenVINS drift internally. The bad-VIO segment demonstrates why Phase 2 should investigate marker updates inside OpenVINS:

- directly correct EKF pose
- correct or reset velocity when appropriate
- update covariance consistently
- use marker measurement covariance and innovation gating
- avoid preserving false high velocity after reanchor

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


## Documentation Handoff

- Detailed user-facing replay commands are documented in
  `multi_cam_localization/sensor_fusion_bringup/docs/head_openvins_marker_bag_replay_validation.md`.
- The main marker validation doc summarizes this replay result in
  `multi_cam_localization/sensor_fusion_bringup/docs/marker_correction_validation.md`.
- Future covariance planning lives in `.agents/marker_pose_covariance_plan.md`.
- Commit source/docs/tests/agent notes only.
- Do not commit generated `build_overlay/`, `install_overlay/`, or
  `log_overlay/` artifacts.
