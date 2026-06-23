# Fix: OpenVINS Arm Node Crash — calib_imu_intrinsics YAML Parse Failure

## Objective

Fix the SIGSEGV crash (exit code -11) in the arm OpenVINS node (`run_subscribe_msckf_marker-9`) caused by the OpenCV FileStorage parser failing to parse `calib_imu_intrinsics: false` from the arm estimator config YAML. The head node works fine because its config was already `false` from the start; the arm config was edited from `true` to `false` with a long comment, which triggers a parsing edge case.

## Root Cause Analysis

The crash chain:
1. `aruco_marker_pose_node.py` → OpenVINS C++ node starts
2. `StateOptions.h:118` calls `parser->parse_config("calib_imu_intrinsics", do_calib_imu_intrinsics)`
3. No ROS2 parameter override exists → falls through to YAML parsing
4. `opencv_yaml_parse.h:392-439` bool parser: `isInt()` check fails, string read returns `[]` (empty sequence)
5. Prints `invalid boolean type of []`, sets `all_params_found_successfully = false`
6. `run_subscribe_msckf_marker.cpp:74-76` calls `std::exit(EXIT_FAILURE)` 
7. Class_loader cleanup during forced exit causes SIGSEGV (-11)

The head config (`head_d435i_336222071386/estimator_config.yaml:16`) has the same `false` value but was never edited — it parsed fine before and after. The arm config was edited from `true` to `false` with a long comment containing special characters (`(0.4 m/s^2)`, etc.). The OpenCV FileStorage `%YAML:1.0` format parser appears to misparse the edited line, returning an empty sequence `[]` instead of the string `"false"`.

## Implementation Plan

- [x] Task 1. Change `calib_imu_intrinsics: false` to `calib_imu_intrinsics: 0` in the arm estimator config. The OpenCV FileStorage parser handles integer `0` via `isInt() == 0` → `false` at `opencv_yaml_parse.h:411-413`, completely bypassing the string parsing path that fails. Shorten the comment to match the head config style to eliminate any possibility of comment-related parsing issues.

  **File**: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/arm_d435i_310622071850/estimator_config.yaml:16`

  **Current**:
  ```yaml
  calib_imu_intrinsics: false # DISABLED: online intrinsic estimation corrupted accel bias (0.4 m/s^2), causing arm divergence. Head camera uses false and is stable. Offline Kalibr calibration is sufficient.
  ```

  **Change to**:
  ```yaml
  calib_imu_intrinsics: 0 # disabled: online intrinsic estimation caused arm divergence; matches head config
  ```

- [x] Task 2. Verify the head config is unaffected (it already uses `false` and works). No change needed — leave as-is for consistency with upstream OpenVINS configs.

  **File**: `multiview_prosthesis-jetson_docker/docker_ws/multi_cam_localization/sensor_fusion_bringup/config/openvins/head_d435i_336222071386/estimator_config.yaml:16`

## Verification Criteria

- [ ] Arm OpenVINS node (`run_subscribe_msckf_marker-9`) starts without `invalid boolean type` error
- [ ] Arm OpenVINS node does not crash with exit code -11
- [ ] Both head and arm OpenVINS nodes reach `odom_ready` phase
- [ ] `calib_imu_intrinsics` is effectively `false` (disabled) for both cameras — verified by checking that IMU intrinsic calibration state elements are NOT created (no `ItoT` or `Tg` in state)

## Potential Risks and Mitigations

1. **Integer `0` vs `false` semantic difference**
   Mitigation: The OpenVINS parser explicitly handles `isInt() == 0` → `false` at `opencv_yaml_parse.h:411-413`. This is the exact same code path used by all upstream OpenVINS configs that use `false` — the parser converts both representations to the same `bool false` value. No semantic difference.

2. **Comment text still causing issues**
   Mitigation: The shortened comment avoids special characters (`(`, `)`, `/`, `^`) that may have confused the OpenCV FileStorage parser. If the issue persists, the comment can be removed entirely.

## Alternative Approaches

1. **Remove the comment entirely**: `calib_imu_intrinsics: 0` with no comment. Simplest, but loses context for future developers.
2. **Use ROS2 parameter override**: Add `calib_imu_intrinsics` as a ROS2 parameter in the launch file. More complex and doesn't fix the root YAML parsing issue.
3. **Rebuild with a patch to the OpenCV FileStorage parser**: Overkill for this issue.
