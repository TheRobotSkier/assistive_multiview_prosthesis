# Dynamic ID2 Arm Update Live Bag Recommendations

This note summarizes the live bags that motivated the final dynamic ID2 arm
update workflow. Dynamic ID2 stays separate from the fixed ID0 marker-map path:
do not add marker ID2 to `marker_fixed_ids`.

## Bags Reviewed

```text
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260512_124920
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260512_132629
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260513_095416
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260513_125316
```

## Current Recommendation

Use high-rate head odometry from `/ov_msckf/odomimu` for the dynamic measurement
producer. The earlier idea of queueing pending ID2 observations is superseded:
the final implementation should not hold old ID2 observations waiting for future
head poses, because delayed EKF updates can make the arm state worse.

The measurement producer now matches each ID2 observation only against head poses
already buffered at callback time:

```text
head_pose_topic: /ov_msckf/odomimu
head_pose_message_type: odometry
max_head_pose_dt_s: 0.05
```

If no already-buffered head pose matches, the observation is rejected immediately
with `no_head_pose_match`. Status includes the signed
`head_pose_time_offset_s`, so it is easy to see whether accepted measurements
used a slightly older or newer already-buffered head pose.

The one-command workflow now defaults to `active`: fixed ID0 updates/reanchor
remain enabled, and dynamic ID2 normal updates plus guarded dynamic ID2
initial-lock/reanchor are enabled. Keep `mode:=observe`, `mode:=update`,
`mode:=would_reanchor`, and `use_dynamic_arm_pose_updates:=false` as the
rollback/diagnostic controls.

## Bag 124920

Key facts:

```text
duration: 85.70 s
messages: 106966
/head/marker_pose/observation: 774
/arm/marker_pose/observation: 0
/head/marker_pose/dynamic_observation: 149
/arm/marker_pose/dynamic_arm_pose_observation: 23
```

Dynamic ID2 quality was good:

```text
ID2 observations: 149
stable ID2 observations: 142
median reprojection: 0.256 px
p95 reprojection: 0.547 px
median distance: 0.587 m
p95 view angle: 33.01 deg
```

The arm fixed ID0 path had no observations, so normal dynamic EKF updates were
blocked by `marker_map_not_locked`. This bag proves the application needs a
separate, guarded dynamic initial-lock path for the arm OpenVINS instance.

## Bag 132629

This bag showed that the dynamic update can help when it is accepted: 15 dynamic
updates applied with small innovations. After a 16.85 s dynamic-measurement gap,
the arm drifted enough that later ID2 measurements were rejected by chi2.

The right response is not to loosen the normal EKF gates broadly. Keep normal
dynamic updates conservative for small innovations, and use a separate dynamic
initial-lock/reanchor evaluator for large disagreements.

## Bag 20260513_095416

This active-mode recording did not validate dynamic ID2 because no ID2 was
published:

```text
/head/marker_pose/dynamic_observation: 0
/arm/marker_pose/dynamic_arm_pose_observation: 0
/ov_msckf_arm/dynamic_arm_update/status: 0
```

Both fixed marker paths stayed ID0-only and odometry was finite, but the
measurement node status was dominated by `head_pose_covariance_too_large`.

## Bag 20260513_125316

This would-reanchor recording validated ID2 detection and measurement
production:

```text
/head/marker_pose/dynamic_observation: 218
/arm/marker_pose/dynamic_arm_pose_observation: 176
/ov_msckf_arm/dynamic_arm_update/status: 0
```

Dynamic ID2 quality was good: median reprojection `0.230 px`, p95 reprojection
`0.555 px`, median distance `0.597 m`, and p95 view angle `47.4 deg`.
Measurement timing was acceptable: median head-pose match dt `0.0106 s`, p95
`0.0413 s`.

This bag did not validate arm OpenVINS update/reanchor because arm odometry
stopped before ID2 measurements began:

```text
/ov_msckf_arm/odomimu ended: 1778676842.1644704
ID2 measurements started:  1778676850.8630111
```

Before final judgment of active behavior, record a bag where
`/ov_msckf_arm/odomimu`, `/arm/marker_pose/dynamic_arm_pose_observation`, and
`/ov_msckf_arm/dynamic_arm_update/status` overlap in time.

## Final Behavior To Validate

Use the one-command launch in this order when revalidating a new setup:

```text
observe -> update -> would_reanchor -> active
```

Expected improvements on replay:

- Bag `124920`: dynamic arm pose observations should increase substantially
  from 23, with strict `0.05 s` matching and no queued stale observations.
- Bag `132629`: dynamic arm pose observations should increase substantially
  from 43, with accepted normal updates staying below `0.35 m` and `15 deg`.
- `would_reanchor` should report plausible dynamic initial-lock/reanchor
  candidates instead of silent `marker_map_not_locked` or repeated chi2
  rejection.
- The default `active` workflow should have no non-finite state, no reset loop,
  no frame collision, and no ID2 entry in `marker_fixed_ids`.

## Tuning Guidance

Start conservative:

```text
dynamic_arm_noise_multiplier: 4.0
dynamic_arm_time_tolerance_s: 0.05
dynamic_arm_max_update_translation_m: 0.35
dynamic_arm_max_update_rotation_deg: 15.0
dynamic_arm_reanchor_measurement_only: true
```

Only tune after reading status from both the measurement node and OpenVINS:

- If `no_head_pose_match` remains high while `/ov_msckf/odomimu` is healthy,
  inspect `head_pose_time_offset_s` before relaxing timing.
- If `dynamic_marker_not_stable` dominates, consider lowering marker stability
  requirements slightly.
- If normal update chi2 rejects after a long gap, prefer `would_reanchor` and
  then guarded dynamic reanchor over increasing the normal EKF gate.
- Fixed ID0 remains higher priority. Dynamic updates and reanchor are skipped
  for the configured time after accepted arm ID0 updates.

## Success Criteria

- Fixed marker observations remain ID0-only.
- Dynamic observations and dynamic arm pose observations remain ID2-only.
- Dynamic measurements are finite, frame-correct, and not delayed by a pending
  queue.
- Normal EKF updates are bounded and explainable in status.
- Dynamic initial-lock/reanchor only occurs in explicit modes with enough
  consistent samples.
- `marker_map -> head_imu -> head_cam0` and
  `marker_map -> arm_imu -> arm_cam0` remain separate and intact.
