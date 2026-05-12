# Dynamic ID2 Arm Update Live Bag Recommendations

Bag analyzed:

```text
docker_ws/bags/openvins_tests/phase2_live/dynamic_id2_arm_update_live_20260512_124920
```

## Summary

The live run confirms that dynamic ID2 is useful for the arm D435i, but the current implementation is too conservative for the actual application:

- The arm D435i saw no fixed ID0 observations, so the current `marker_map_not_locked` guard made dynamic updates impossible.
- ID2 quality was good; the low dynamic update rate was mostly a head-pose synchronization issue, not bad marker detection.
- The unanchored arm OpenVINS pose diverged far enough that dynamic ID2 should be allowed to perform a controlled reanchor/initial lock, not only a small EKF correction.

Do not solve this by adding ID2 to `marker_fixed_ids`. Keep ID2 in a separate dynamic-arm path.

## Bag Facts

Bag metadata:

```text
duration: 85.70 s
size: 4.1 GiB
messages: 106966
```

Topic counts of interest:

```text
/head/marker_pose/observation: 774
/arm/marker_pose/observation: 0
/head/marker_pose/dynamic_observation: 149
/arm/marker_pose/dynamic_arm_pose_observation: 23
/arm/marker_pose/dynamic_arm_measurement/status: 149
/ov_msckf_arm/dynamic_arm_update/status: 23
```

Dynamic ID2 quality:

```text
ID2 observations: 149
stable ID2 observations: 142
median reprojection: 0.256 px
p95 reprojection: 0.547 px
median distance: 0.587 m
p95 view angle: 33.01 deg
```

Measurement producer status:

```text
accepted_published: 23
no_head_pose_match: 119
dynamic_marker_not_stable: 7
```

Arm OpenVINS dynamic update status:

```text
marker_map_not_locked: 21
visualizer_stale: 2
accepted: 0
state_updated: 0
```

The OpenVINS dynamic status showed `measurement_only:false`, so this was a real-update attempt, but every update was rejected before innovation because the arm instance never locked to `marker_map`.

## Why Updates Were Infrequent

Offline synchronization shows that most stable ID2 observations had a suitable head pose nearby:

```text
stable ID2 with nearest head pose <= 0.05 s: 132 / 142
```

But in live message order, only a small fraction had a head pose already available when the dynamic ID2 message arrived:

```text
latest already-received head pose within 0.05 s: 24 / 142
latest already-received head pose within 0.10 s: 60 / 142
latest already-received head pose within 0.20 s: 82 / 142
```

Recommendation:

- Add a small pending dynamic-observation queue in `dynamic_arm_pose_measurement_node.py`.
- When an ID2 observation arrives without a matching head pose, keep it briefly instead of immediately rejecting it.
- Retry pending observations whenever a new head pose arrives.
- Keep the actual accepted sync tolerance tight, for example 0.05 s, after the matching pose arrives.
- Add status fields for queued, expired, matched-after-wait, and wait duration.

This should recover many of the missed measurements without using stale head poses.

## Why Dynamic Reanchor Is Needed

The arm fixed ID0 path did not receive observations in this live bag:

```text
/arm/marker_pose/observation: 0
```

So the current rule, "arm fixed ID0 must lock marker_map before dynamic ID2 can update," is incompatible with the live application.

The dynamic measurements also disagreed strongly with the current arm OpenVINS pose:

```text
matched dynamic measurements vs arm pose: 22
median translation residual: 147.30 m
median rotation residual: 152.48 deg
```

That means the arm estimator was effectively unanchored in `marker_map`. A normal EKF update should not be stretched to handle this; the dynamic path needs its own controlled reset/reanchor mode.

## Recommended Next Implementation

Add an explicit dynamic-arm reanchor path, default disabled:

```text
dynamic_arm_allow_initial_lock: false
dynamic_arm_allow_reanchor: false
dynamic_arm_reanchor_measurement_only: true
dynamic_arm_reanchor_min_samples: 5
dynamic_arm_reanchor_window_s: 0.75
dynamic_arm_reanchor_min_sample_dt_s: 0.10
dynamic_arm_reanchor_max_velocity_mps: 2.0
dynamic_arm_reanchor_max_sample_translation_std_m: 0.10
dynamic_arm_reanchor_max_sample_rotation_std_deg: 8.0
dynamic_arm_reanchor_cooldown_s: 3.0
```

Behavior:

- Keep dynamic ID2 out of `marker_fixed_ids`.
- Keep dynamic reanchor separate from the fixed ID0 marker-map updater.
- Allow dynamic ID2 to perform the first arm `marker_map` lock only when `dynamic_arm_allow_initial_lock:=true`.
- Allow later dynamic reanchor only when `dynamic_arm_allow_reanchor:=true`.
- Require a short history of stable, finite, frame-correct dynamic arm pose measurements.
- Fit velocity from the dynamic measurement history, analogous to the fixed marker reset path.
- Reset only the arm OpenVINS instance to `T_map_armimu` from the dynamic measurement.
- Do not reset or reanchor the head OpenVINS instance.
- Do not use dynamic updates in fixed marker reset history.
- Fixed ID0, when available, should remain higher priority.
- Keep the skip-after-fixed-marker gate and log `skipped_recent_fixed_marker_update`.

The first implementation should support a reanchor measurement-only mode that logs "would reanchor" with the proposed pose, covariance, velocity fit, and reason before mutating state.

## Gate Recommendations

Do not loosen the main innovation gates first. The present bottlenecks are:

1. The missing dynamic observation wait queue.
2. The hard `marker_map_not_locked` dependency on arm ID0.

After those are addressed, tune conservatism from status data:

- Keep `dynamic_arm_noise_multiplier:=4.0` initially.
- Keep `dynamic_arm_time_tolerance_s:=0.05` for accepted matched poses.
- Keep `dynamic_arm_max_update_translation_m:=0.35` and `dynamic_arm_max_update_rotation_deg:=15.0` for normal EKF updates.
- Use the new dynamic reanchor path for larger residuals.
- Consider reducing `stable_frames_required` from 8 to 5 only if future bags show missed opportunities due to stability, not synchronization.

In this bag, only 7 messages were rejected for `dynamic_marker_not_stable`, so stability was not the main rate limiter.

## Validation Order

1. Implement pending dynamic-observation sync in the measurement node.
2. Replay this live bag and verify dynamic arm pose observations increase substantially.
3. Implement dynamic reanchor in measurement-only mode.
4. Replay this live bag and verify status reports plausible `would_dynamic_reanchor` events.
5. Enable dynamic reanchor in replay and verify no non-finite state, no frame collisions, and no repeated reset loop.
6. Run live with dynamic reanchor measurement-only first.
7. Enable live dynamic reanchor only after status and RViz behavior are explainable.

Success should be status-focused:

- More dynamic measurements from the same ID2 observations.
- Clear accepted/rejected/queued/expired reasons.
- Dynamic reanchor only after enough consistent samples.
- No snaps from normal EKF updates.
- No ID2 entry in `marker_fixed_ids`.
- Fixed ID0 behavior preserved when ID0 is visible.
