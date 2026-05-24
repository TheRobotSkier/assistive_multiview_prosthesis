# Fix Node Startup Crashes: Parameter Type Mismatches

**Date:** 2026-05-24
**Status:** Ready for Implementation
**Severity:** CRITICAL — both nodes crash immediately, breaking the entire perception pipeline

---

## Objective

Fix two parameter type mismatches that cause `openvins_odom_tf_relay` and `segmentation_ros2_node` to crash on startup, as observed in `camera-test-log-v7.txt`. These crashes are the root cause of the fused point cloud "jumping" behavior — the relay crash severs the head camera TF chain, leaving only the arm camera to publish (with continuous VIO updates causing visible jitter).

---

## Context

### Crash 1: `openvins_odom_tf_relay` (lines 41-64 of v7 log)

```
TypeError: The given value is not one of the allowed types 
'{'x': 0.0, 'y': 0.015, 'z': 0.0, 'roll': -1.57079632679, 'pitch': 0.0, 'yaw': -1.57079632679}'
```

ROS2 Jazzy's `declare_parameter()` does not support `dict` as a parameter type. The node at `src/camera/camera/openvins_odom_tf_relay.py:106-114` declares `imu_to_cam_extrinsics_head` and `imu_to_cam_extrinsics_arm` with a dict default value. The config at `config/prosthesis_config.yaml:163-166` also passes a dict. Both paths fail.

### Crash 2: `segmentation_ros2_node` (lines 65-82 of v7 log)

```
InvalidParameterTypeException: 
Trying to set parameter 'roi_radius_m' to '0.3' of type 'STRING', expecting type 'DOUBLE'
```

The launch file at `src/prosthesis_launch/launch/pipeline.launch.py:304` passes `roi_radius` from `LaunchConfiguration`, which always returns a **string**. The segmentation node at `src/segmentation/segmentation_bridge/segmentation_ros2_node.py:119` declares it as a DOUBLE. The type mismatch causes a crash.

---

## Implementation Plan

### Phase 1: Fix `openvins_odom_tf_relay` dict parameter

Replace the dict parameters with six individual `double` parameters per camera (head/arm). This is the most robust approach — ROS2 natively supports `double` parameters, and individual parameters are self-documenting and easy to override from the command line.

- [ ] **1.1** In `src/camera/camera/openvins_odom_tf_relay.py:106-115`, replace the two dict parameter declarations with 12 individual double parameters:

  Replace:
  ```python
  self.declare_parameter(
      "imu_to_cam_extrinsics_head",
      {"x": 0.0, "y": 0.015, "z": 0.0,
       "roll": -1.57079632679, "pitch": 0.0, "yaw": -1.57079632679},
  )
  self.declare_parameter(
      "imu_to_cam_extrinsics_arm",
      {"x": 0.0, "y": 0.015, "z": 0.0,
       "roll": -1.57079632679, "pitch": 0.0, "yaw": -1.57079632679},
  )
  ```

  With:
  ```python
  # Head IMU->cam0 extrinsic (nominal D435i: color_optical is (0, 0.015, 0)
  # from link origin, with optical rotation rpy=(-pi/2, 0, -pi/2)).
  self.declare_parameter("imu_to_cam_x_head", 0.0)
  self.declare_parameter("imu_to_cam_y_head", 0.015)
  self.declare_parameter("imu_to_cam_z_head", 0.0)
  self.declare_parameter("imu_to_cam_roll_head", -1.57079632679)
  self.declare_parameter("imu_to_cam_pitch_head", 0.0)
  self.declare_parameter("imu_to_cam_yaw_head", -1.57079632679)
  # Arm IMU->cam0 extrinsic (same nominal values as head).
  self.declare_parameter("imu_to_cam_x_arm", 0.0)
  self.declare_parameter("imu_to_cam_y_arm", 0.015)
  self.declare_parameter("imu_to_cam_z_arm", 0.0)
  self.declare_parameter("imu_to_cam_roll_arm", -1.57079632679)
  self.declare_parameter("imu_to_cam_pitch_arm", 0.0)
  self.declare_parameter("imu_to_cam_yaw_arm", -1.57079632679)
  ```

  **Rationale:** ROS2 Jazzy supports `double` parameters natively. Individual parameters are unambiguous, type-safe, and can be overridden individually from the CLI or launch files.

- [ ] **1.2** In `src/camera/camera/openvins_odom_tf_relay.py:126-127`, update the parameter reading to build dicts from the individual parameters:

  Replace:
  ```python
  head_extrinsics = self.get_parameter("imu_to_cam_extrinsics_head").value
  arm_extrinsics = self.get_parameter("imu_to_cam_extrinsics_arm").value
  ```

  With:
  ```python
  head_extrinsics = {
      "x": self.get_parameter("imu_to_cam_x_head").value,
      "y": self.get_parameter("imu_to_cam_y_head").value,
      "z": self.get_parameter("imu_to_cam_z_head").value,
      "roll": self.get_parameter("imu_to_cam_roll_head").value,
      "pitch": self.get_parameter("imu_to_cam_pitch_head").value,
      "yaw": self.get_parameter("imu_to_cam_yaw_head").value,
  }
  arm_extrinsics = {
      "x": self.get_parameter("imu_to_cam_x_arm").value,
      "y": self.get_parameter("imu_to_cam_y_arm").value,
      "z": self.get_parameter("imu_to_cam_z_arm").value,
      "roll": self.get_parameter("imu_to_cam_roll_arm").value,
      "pitch": self.get_parameter("imu_to_cam_pitch_arm").value,
      "yaw": self.get_parameter("imu_to_cam_yaw_arm").value,
  }
  ```

  **Rationale:** The downstream code at lines 155-172 already accesses extrinsics via `extrinsics["x"]`, `extrinsics["roll"]`, etc. Building a dict from the individual parameters preserves compatibility with all downstream usage without changing any other code.

- [ ] **1.3** In `config/prosthesis_config.yaml:163-166`, replace the dict parameters with flat double parameters:

  Replace:
  ```yaml
  imu_to_cam_extrinsics_head:
      {x: 0.0, y: 0.015, z: 0.0, roll: -1.570796, pitch: 0.0, yaw: -1.570796}
  imu_to_cam_extrinsics_arm:
      {x: 0.0, y: 0.015, z: 0.0, roll: -1.570796, pitch: 0.0, yaw: -1.570796}
  ```

  With:
  ```yaml
  # Nominal D435i imu->cam0 (color_optical) extrinsic.
  # OpenVINS cam0 == color_optical_frame.  The imu frame is at the link
  # origin; color_optical is (0, 0.015, 0) with optical rotation.
  imu_to_cam_x_head: 0.0
  imu_to_cam_y_head: 0.015
  imu_to_cam_z_head: 0.0
  imu_to_cam_roll_head: -1.570796
  imu_to_cam_pitch_head: 0.0
  imu_to_cam_yaw_head: -1.570796
  imu_to_cam_x_arm: 0.0
  imu_to_cam_y_arm: 0.015
  imu_to_cam_z_arm: 0.0
  imu_to_cam_roll_arm: -1.570796
  imu_to_cam_pitch_arm: 0.0
  imu_to_cam_yaw_arm: -1.570796
  ```

  **Rationale:** Must match the new parameter names from step 1.1. YAML float values will be correctly parsed as Python floats and passed to ROS2 as DOUBLE types.

- [ ] **1.4** Update the docstring at `src/camera/camera/openvins_odom_tf_relay.py:27-28` to reflect the new parameter names:

  Replace:
  ```
  imu_to_cam_extrinsics_head  dict  nominal head imu->cam0 {x,y,z,roll,pitch,yaw}
  imu_to_cam_extrinsics_arm   dict  nominal arm  imu->cam0 {x,y,z,roll,pitch,yaw}
  ```

  With:
  ```
  imu_to_cam_{x,y,z,roll,pitch,yaw}_head  double  nominal head imu->cam0 extrinsic components
  imu_to_cam_{x,y,z,roll,pitch,yaw}_arm   double  nominal arm  imu->cam0 extrinsic components
  ```

  **Rationale:** Keep documentation in sync with code.

### Phase 2: Fix `segmentation_ros2_node` string-to-double parameter

- [ ] **2.1** In `src/prosthesis_launch/launch/pipeline.launch.py:304`, cast `roi_radius` to float before passing it as a parameter:

  Replace:
  ```python
  parameters=[{"inference_url": inference_url, "roi_radius_m": roi_radius}],
  ```

  With:
  ```python
  parameters=[{"inference_url": inference_url, "roi_radius_m": float(roi_radius)}],
  ```

  **Rationale:** `LaunchConfiguration.perform()` always returns a string. The segmentation node declares `roi_radius_m` as DOUBLE (`declare_parameter("roi_radius_m", 0.3)`). A simple `float()` cast converts the string `"0.3"` to the Python float `0.3`, which ROS2 will accept as a DOUBLE parameter. This is a one-line fix with no other changes needed.

---

## Files Changed

| File | Change |
|------|--------|
| `src/camera/camera/openvins_odom_tf_relay.py` | Replace 2 dict params with 12 individual double params; update param reading to build dicts; update docstring |
| `config/prosthesis_config.yaml` | Replace 2 dict entries with 12 flat double entries under `openvins_odom_tf_relay` |
| `src/prosthesis_launch/launch/pipeline.launch.py` | Cast `roi_radius` to `float()` on line 304 |

---

## Verification Criteria

- [ ] `openvins_odom_tf_relay` starts without crash and logs: `OpenVINS odometry TF relay active: marker_map -> head_imu ...`
- [ ] `segmentation_ros2_node` starts without crash and logs its ready message
- [ ] `tf_pipeline_diagnostics` reports `OpenVINS(head):OK` and `OpenVINS(arm):OK` within 30 seconds of startup
- [ ] Fusion node stats show `dual > 0` (both cameras fusing) instead of `dual=0, cam1_only=N`
- [ ] Bbox removal success rate is > 90% (no more "0% over Ns" errors)
- [ ] No `TypeError` or `InvalidParameterTypeException` in the log

---

## Potential Risks and Mitigations

1. **Existing parameter overrides break if anyone uses the old dict parameter names on the CLI**
   - **Risk level:** Low — the old dict params never worked (they always crashed), so no one could be relying on them in working configs.
   - **Mitigation:** The new flat parameters have clear, self-documenting names. The config file change ensures the default values are correct.

2. **YAML float precision for the extrinsic values**
   - **Risk level:** Very low — the values are simple floats (0.0, 0.015, -1.570796). YAML parses these correctly as Python floats.
   - **Mitigation:** The node's `declare_parameter()` defaults use full-precision values (-1.57079632679). The config uses -1.570796 which is rounded but close enough. If exact precision matters, update the config values to match the code defaults.

3. **`float()` cast fails if `roi_radius` launch arg is empty or non-numeric**
   - **Risk level:** Very low — the launch argument has a default value of `"0.3"` and is only overridden via the CLI with numeric values.
   - **Mitigation:** If an invalid value is passed, the `float()` call will raise a clear `ValueError` at launch time, which is much better than the current silent crash inside the node.

---

## Alternative Approaches

1. **Use `string` parameter type + JSON parsing for the extrinsics**: Declare the parameter as a string containing JSON (e.g., `'{"x": 0.0, "y": 0.015, ...}'`), then `json.loads()` inside the node. This preserves the dict structure but adds a parsing dependency and is less idiomatic for ROS2.
   - **Trade-off:** More compact config, but fragile (JSON syntax errors at runtime) and not individually overridable from the CLI.

2. **Use `double_array` parameter type**: Declare as `list[float]` with 6 elements in a fixed order `[x, y, z, roll, pitch, yaw]`. This is a single parameter per camera.
   - **Trade-off:** Compact, but positional parameters are error-prone (easy to swap roll/pitch/yaw). Individual named parameters are clearer and safer.

3. **Fix the segmentation issue by changing the node to accept string**: Have the node call `float()` on the parameter value itself instead of relying on ROS2 type inference.
   - **Trade-off:** Moves the fix to the wrong layer. The launch file should pass correctly-typed values; the node shouldn't need to work around launch-file type coercion.
