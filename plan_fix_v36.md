# Plan to Fix v36 Runtime Issues

Sources used:

- `plans/2026-05-26-2026-05-26-log-v36-diagnostic-report-v1.md`
- Targeted searches in `logs/host-log-v36.txt`
- Current codebase files named below

I did not inspect older plans or older logs.

## Summary

The main failure chain is:

1. A proximity plan is committed during attempt 1 and is never cleared.
2. The proximity controller keeps publishing finger and wrist commands outside `APPROACHING`.
3. Attempt 2 preshaping fails because the contact override expires before segmentation finishes.
4. The pipeline returns to `IDLE`, but twist propagation is not deactivated, so it keeps triggering segmentations.
5. The stale proximity plan then reacts to movement and enters the near zone, but the pipeline is no longer in `APPROACHING`, so the hand never transitions to `GRASPING`.

The wrist issue is separate but safety-critical: the planner samples signed wrist deltas, but `c_api.rs` wraps negative angles into `[0, 360)`, and the wrist driver interprets the value as an absolute Dynamixel target.

## Fix 1: Make proximity execution state-owned

Evidence:

- `logs/host-log-v36.txt:462` commits a plan with `wrist=277.2`.
- `logs/host-log-v36.txt:507` transitions `APPROACHING -> RELEASING`.
- `logs/host-log-v36.txt:516` transitions `RELEASING -> IDLE`.
- `logs/host-log-v36.txt:518-532` still publishes `FAR mode`.
- `logs/host-log-v36.txt:705` enters near zone while not in `APPROACHING`.

Root cause:

- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:201-202` only stores pipeline state.
- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:231-291` runs in every state except `GRASPING`, `HOLDING`, and `VOLITIONAL`.
- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:222-228` publishes initial commands immediately on plan commit, while the pipeline is still in `PLANNING`.

Fix:

- In `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`, add explicit state constants matching `pipeline_manager_node.py`.
- Add `_clear_plan(reason, clear_buffers=True)` that clears `_planned_closures`, `_planned_wrist_deg`, `_planned_hand_frame`, `_buf_*`, resets `_is_near`, and publishes `Bool(False)` if the near-zone latch was true.
- Change `_on_pipeline_state` to track previous state and call `_clear_plan(...)` when leaving `APPROACHING` or entering `IDLE`, `TWISTING`, `SEGMENTING`, `PLANNING`, or `RELEASING`.
- Change `_control_loop` to return unless `self._pipeline_state == APPROACHING`.
- Remove the immediate command publish from `_try_commit_plan`, or guard it with `if self._pipeline_state == APPROACHING`. Prefer removal: the timer should be the only command source.

Verification:

- Add a unit-level test or small ROS integration test that publishes a plan, then publishes pipeline state `IDLE`, and verifies no finger/wrist commands are emitted.
- Re-run the pipeline and confirm no `FAR mode` or `NEAR mode` logs appear outside `APPROACHING`.

## Fix 2: Add proximity distance sanity guards

Evidence:

- `logs/host-log-v36.txt:1060` reports `dist=0.950 m`.
- `logs/host-log-v36.txt:1065` reports `dist=5.791 m`.
- `logs/host-log-v36.txt:1111` reports `dist=930.350 m`.

Root cause:

- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:256-291` computes distance and publishes commands without checking finite values or plausible workspace bounds.
- `src/camera/camera/openvins_odom_tf_relay.py:313-378` filters initialization garbage, but after initialization it keeps publishing TF even if OpenVINS diverges.

Fix:

- In `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`, add parameters:
  - `max_valid_proximity_distance_m`, default `0.75`
  - `max_proximity_step_m`, default `0.25`
- If distance is non-finite, greater than `max_valid_proximity_distance_m`, or jumps by more than `max_proximity_step_m` since the previous valid sample, call `_clear_plan("invalid proximity distance")` and publish no commands.
- In `config/prosthesis_config.yaml`, add these parameters under `proximity_controller.ros__parameters`.
- In `src/camera/camera/openvins_odom_tf_relay.py`, add post-initialization outlier suppression:
  - `max_pose_norm_m`, default `2.0`
  - `max_pose_jump_m`, default `0.50`
  - Do not broadcast TF for outlier odometry; log throttled warnings.

Verification:

- Inject an odom pose jump in a local test and confirm no TF is broadcast after the jump.
- Inject a hand pose that makes proximity distance exceed `0.75 m` and confirm the proximity node clears the plan and sends no commands.

## Fix 3: Keep contact override valid through segmentation

Evidence:

- Attempt 1 logs `Planner called with contact override` at `logs/host-log-v36.txt:443` and succeeds.
- Attempt 2 has a hit at `logs/host-log-v36.txt:556-560`, segmentation publishes at `logs/host-log-v36.txt:568`, but there is no `Planner called with contact override` line before the failure at `logs/host-log-v36.txt:573`.
- Attempt 2 segmentation took just over 2 seconds, so the hard-coded 2 second contact override freshness expired.

Root cause:

- `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:319-327` accepts the contact override only if the contact pose is younger than `rclcpp::Duration(2, 0)`.
- If the override expires, the Rust planner uses the live hand pose to predict the ROI, while the segmented cloud is around the contact click. That mismatch leads to `No points in ROI`.

Fix:

- In `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`, replace the hard-coded `2` second override age with a declared parameter `contact_override_max_age_s`.
- Set `contact_override_max_age_s: 6.0` in `config/prosthesis_config.yaml` under `preshaping_service.ros__parameters`.
- Log the contact override age when used or rejected, so future logs show whether preshaping is anchored at the click or at the live hand pose.
- Longer-term robust option: have `pipeline_manager_node.py` latch the hit/contact pose at `TWISTING -> SEGMENTING` and include that specific hit identity in the preshaping request. The parameterized age fix is the smaller immediate repair.

Verification:

- Simulate a 2.5 second segmentation delay and confirm preshaping still logs `Planner called with contact override`.
- Confirm the `No points in ROI` failure no longer occurs for segmented clouds with nonzero ROI crop counts.

## Fix 4: Deactivate twist propagation on all failure exits

Evidence:

- `logs/host-log-v36.txt:573` preshaping fails.
- `logs/host-log-v36.txt:574` transitions `PLANNING -> IDLE`.
- `logs/host-log-v36.txt:591`, `639`, `682`, `795`, `922`, `947`, and `1040` show later hits, so twist propagation remained active.

Root cause:

- `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:636-638` deactivates twist only on successful preshaping.
- `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:639-644` failure and exception paths transition to `IDLE` without calling `_deactivate_twist_propagation()`.
- `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py:625-628` service-unavailable path also transitions to `IDLE` without twist cleanup.

Fix:

- In `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`, call `_deactivate_twist_propagation()` in every path that transitions from `TWISTING`, `SEGMENTING`, or `PLANNING` to `IDLE`.
- Prefer centralizing this in `_transition`: after publishing the state, if `new_state in (IDLE, RELEASING)` and `old != IDLE`, call `_deactivate_twist_propagation()`. Keep the existing explicit success deactivation on `PLANNING -> APPROACHING`.
- Clear `_segmenting_start_time` when leaving `SEGMENTING` or `PLANNING`.

Verification:

- Mock the preshaping service to return `success=false` and verify the twist deactivate service is called once.
- Re-run and confirm no twist `Hit found` logs occur after `PLANNING -> IDLE`.

## Fix 5: Make wrist command semantics safe

Evidence:

- `logs/host-log-v36.txt:462` commits `wrist=277.2`.
- `config/grasp_preshaping.yaml:24` limits sampled wrist rotation to `pi/2`, so `277.2 deg` is likely a wrapped `-82.8 deg`, not a valid intended absolute target.

Root cause:

- `src/grasp_preshaping/src/c_api.rs:605` converts the sampled wrist rotation with `.rem_euclid(360.0)`, turning negative deltas into large positive angles.
- `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp:541-543` publishes this raw value.
- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:351-354` forwards it directly to `/wrist/set_position`.
- `src/wrist_driver/wrist_driver/wrist_driver_node.py:52-54` applies modulo again and interprets the command as an absolute Dynamixel position.

Fix:

- In `src/grasp_preshaping/src/c_api.rs`, change wrist output to signed degrees:
  - Replace `particles[best_idx].wrist_rotation.to_degrees().rem_euclid(360.0)` with `particles[best_idx].wrist_rotation.to_degrees()`.
- In `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`, treat `ffi_response.wrist_rotation_deg` as a relative signed wrist delta. Add `max_wrist_delta_deg`, default `90.0`, and clamp before publishing.
- In `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`, convert the signed planner delta into a safe absolute motor target:
  - Subscribe to `/wrist/state` and cache current wrist angle.
  - At plan commit, compute `target = current_wrist_deg + planned_delta_deg` or `wrist_neutral_deg + planned_delta_deg` if no state exists.
  - Clamp to `wrist_min_deg` and `wrist_max_deg`.
- In `src/wrist_driver/wrist_driver/wrist_driver_node.py`, add hard device-bound parameters `min_position_deg` and `max_position_deg`, and clamp or reject before `_deg_to_dx`. Do not silently wrap unsafe negative or greater-than-360 inputs with modulo.
- Add the wrist safety parameters to `config/prosthesis_config.yaml`.

Verification:

- Add a Rust test that a `-82.8 deg` sampled wrist remains negative in the FFI output.
- Publish `/grasp_preshaping/wrist_pose = -82.8` with `/wrist/state = [47.9, 0.0]` and confirm the proximity controller sends a clamped safe absolute target, not `277.2`.
- Publish an unsafe `/wrist/set_position` command and confirm the wrist driver clamps/rejects it.

## Fix 6: Reduce Mia Hand command spam and harden ACK handling

Evidence:

- `logs/host-log-v36.txt:479-494`, `520-534`, and many later lines show repeated `Invalid ACK` and ACK timeout warnings.
- The proximity node publishes three finger commands every 10 Hz in both FAR and NEAR modes.
- After attempt 1, stale-plan publishing continues for minutes outside `APPROACHING`.

Root cause:

- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:167-168` runs the control timer at 10 Hz.
- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:281` and `286-287` publish repeated identical finger commands every tick.
- `src/command_bridge/command_bridge/command_bridge_node.py:138-173` forwards every command to an async service call with no deduplication, in-flight guard, or minimum interval.
- `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp:1322` waits only 20 ms for ACK and treats any nonmatching serial bytes as invalid ACK.

Fix:

- First apply Fix 1, which stops all command publishing outside `APPROACHING`.
- In `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`, add output command caching:
  - Publish only when FAR/NEAR mode changes, plan changes, target changes beyond an epsilon, or a watchdog repeat period expires.
  - Use `command_repeat_period_s: 1.0` and `command_epsilon: 0.005` as initial parameters.
- In `src/command_bridge/command_bridge/command_bridge_node.py`, add per-finger command coalescing:
  - If a service call is in flight for a finger, store only the latest desired target.
  - Do not send duplicate targets within epsilon.
  - Enforce a per-finger minimum interval, e.g. `0.20 s`.
- In `src/mia_hand_driver/src/mia_hand_driver/cpp_driver.cpp`, add a single retry path for ACK timeout/invalid ACK:
  - Flush input after failure.
  - Re-send once under the same mutex.
  - Increase ACK timeout to a parameterized value, initially `50 ms` or `100 ms`.
- If ACK failures persist after throttling, test whether joint position streaming pollutes the ACK path. The likely test is to disable the stream in `command_bridge_node.py` temporarily and compare ACK failure rate.

Verification:

- Add a `command_bridge` unit test for deduplication and in-flight coalescing.
- In a hardware run, confirm command bridge warning rate drops sharply and hand disconnect/reconnect logs stop during approach.

## Fix 7: Increase near-zone hysteresis and add debounce

Evidence:

- `logs/host-log-v36.txt:1013-1033` alternates near-zone enter/exit multiple times around the `0.08 m` / `0.10 m` thresholds.
- Runtime log `logs/host-log-v36.txt:351` shows `exit_thresh=0.100 m`.

Root cause:

- `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py:59-60` defaults to `0.08` / `0.10`.
- `config/prosthesis_config.yaml:133-134` currently has top-level `0.08` / `0.20`, but `config/prosthesis_config.yaml:452-453` still has duplicate `full_pipeline.execution` values `0.08` / `0.10`.
- The runtime used `0.10`, so either the mounted config was stale or a launch/config path still picked the duplicate/default value.

Fix:

- In `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`, change the default `proximity_exit_threshold_m` to `0.20`.
- In `config/prosthesis_config.yaml`, set every proximity exit threshold to `0.20`, including `full_pipeline.execution`.
- Add debounce in the proximity node:
  - `near_enter_consecutive_samples: 3`
  - `near_exit_consecutive_samples: 3`
  - Only publish `Bool(True)` after 3 consecutive below-enter samples.
  - Only publish `Bool(False)` after 3 consecutive above-exit samples.
- Consider removing the duplicate `full_pipeline.execution` proximity values or generating node params from one canonical section only.

Verification:

- Confirm startup log reports `exit_thresh=0.200 m`.
- Replay or simulate noisy distances around `0.08-0.10 m` and confirm no rapid toggling.

## Fix 8: Make missing head OpenVINS a preflight failure or explicit degraded mode

Evidence:

- `logs/host-log-v36.txt:394-407` shows both head and arm initially waiting.
- `logs/host-log-v36.txt:412` shows only arm initializes.
- Every later relay status reports `head: WAITING (0 msgs), arm: OK`.
- `src/camera/camera/openvins_odom_tf_relay.py:119-120` expects head odom on `/ov_msckf/odomimu` and arm odom on `/ov_msckf_arm/odomimu`.

Root cause:

- The host relay is not receiving head OpenVINS odometry at all. The fault is upstream of `openvins_odom_tf_relay.py`: Jetson OpenVINS launch/topic remap/startup, or a missing head OpenVINS process.
- The host pipeline continues with single-camera pointcloud quality but does not clearly fail or enter an explicit degraded mode.

Fix:

- Add a host preflight script, e.g. `scripts/check_openvins_topics.sh`, that checks:
  - `/head/d435i_head/depth/color/points`
  - `/arm/d435i_arm/depth/color/points`
  - `/ov_msckf/odomimu`
  - `/ov_msckf_arm/odomimu`
- Wire the preflight into `Makefile.workspace` before `make run` or into `src/prosthesis_launch/launch/pipeline.launch.py` as a `require_dual_openvins` launch arg.
- If dual OpenVINS is required, abort before enabling grasp execution when the head odom topic is missing.
- If single-camera mode is acceptable, publish an explicit degraded-mode diagnostic and configure the planner/fusion nodes to ignore the head frame instead of repeatedly warning.
- On the Jetson side, verify the OpenVINS launch creates two independent odom publishers with the expected topic names. The host-side files to align against are `config/prosthesis_config.yaml` and `src/camera/camera/openvins_odom_tf_relay.py`.

Verification:

- Run the preflight with head OpenVINS stopped and confirm the pipeline refuses to start grasp execution.
- Run it with both odom streams active and confirm the relay logs both head and arm initialized.

## Recommended Implementation Order

1. Implement Fix 1 and Fix 2 first. These are the hard safety gates and stop stale commands.
2. Implement Fix 3 and Fix 4 next. These repair attempt 2 and stop runaway segmentation loops.
3. Implement Fix 5 before any further wrist hardware tests.
4. Implement Fix 6 to reduce Mia Hand serial failures.
5. Implement Fix 7 to remove noisy near-zone behavior.
6. Implement Fix 8 as a preflight gate before relying on dual-camera planning.

## Minimal Retest Sequence

1. Local unit tests for proximity state gating, distance guards, and wrist conversion.
2. Local pipeline launch with mock hand/wrist and injected plan/state messages.
3. Hardware dry run with wrist disabled, verifying no command spam outside `APPROACHING`.
4. Hardware run with wrist enabled only after signed/clamped wrist behavior is verified.
5. Full run with both OpenVINS streams confirmed before grasp execution.
