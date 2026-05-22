# Twist Propagation: Start/Stop Flag & Config Hardening

## Objective

Add a proper start/stop (enable/disable) flag to the twist propagation node that can be toggled at runtime, and audit all hardcoded values in the node + launch files to ensure they are driven from configuration files instead of being inline literals.

---

## 1. Initial Assessment

### 1.1 Project Structure Summary

The twist propagation subsystem lives under `src/twist_propagation/` and is consumed by multiple launch files across the workspace:

| Component | Path |
|---|---|
| Node implementation | `src/twist_propagation/twist_propagation/twist_propagation_node.py` |
| Package-local config | `src/twist_propagation/config/twist_propagation.yaml` |
| Central config | `config/prosthesis_config.yaml` |
| Package-local launch | `src/twist_propagation/launch/twist_propagation.launch.py` |
| Pipeline launch | `src/prosthesis_launch/launch/pipeline.launch.py` |
| Test launch | `src/prosthesis_launch/launch/twist_propagation_test.launch.py` |
| Mock launch | `src/prosthesis_launch/launch/mock.launch.py` |
| Grasp test launch | `src/prosthesis_launch/launch/grasp_test.launch.py` |
| Digital twin launch | `src/prosthesis_launch/launch/digital_twin.launch.py` |
| Unit tests | `src/twist_propagation/test/test_twist_propagation.py` |
| Integration tests | `scripts/test_twist_propagation_integration.py` |

### 1.2 Current Start/Stop Mechanism

The node **already has** an activation mechanism:

- **Parameter**: `active` (bool, default `false`) at `twist_propagation_node.py:356`
- **Services**: `/twist_propagation/activate` and `/twist_propagation/deactivate` (std_srvs/Trigger) at lines 487-496
- **Internal state**: `self._active` boolean at line 406
- **Cycle guard**: `_cycle_callback` checks `self._active` at line 971 and returns early if inactive
- **Service callbacks**: `_on_activate` (line 517) and `_on_deactivate` (line 528) toggle the flag, clear state, and reset visualization

**Assessment**: The runtime start/stop mechanism via ROS services is already well-implemented. The `active` parameter controls initial state, and services allow runtime toggling.

### 1.3 Hardcoded Values Found

The following values are hardcoded in the node instead of being configurable:

| Hardcoded Value | Location | Description |
|---|---|---|
| `alpha = 0.4` | `twist_propagation_node.py:640` | EMA smoothing factor for twist estimation |
| `0.06` (6cm sphere) | `twist_propagation_node.py:867` | Hit marker sphere diameter |
| `2` (seconds) | `twist_propagation_node.py:869` | Hit marker persistence lifetime |
| `0.01` (1cm line) | `twist_propagation_node.py:891` | Trajectory line width |
| `50` (max spheres) | `twist_propagation_node.py:833` | Max collision spheres for visualization |
| Visualization topic names | `twist_propagation_node.py:472-484` | 6 hardcoded topic strings for status/path/markers |
| `ColorRGBA` values | `twist_propagation_node.py:847,866,893-896` | Marker colors hardcoded |
| `0.5` (timeout) | `twist_propagation_node.py:772` | TF transform timeout |

### 1.4 Launch File Hardcoding Issues

| Launch File | Issue |
|---|---|
| `pipeline.launch.py:119-125` | Passes entire config file -- OK, but no `active` override |
| `twist_propagation_test.launch.py:98-105` | Passes inline `{"active": active == "true", "input_cloud_topic": ..., "odom_topic": ...}` but NOT the config file -- all other params get defaults from `declare_parameter()` |
| `grasp_test.launch.py:110-123` | Passes inline `{"active": True, "input_cloud_topic": cloud_topic}` only -- no config file, all other params get defaults |
| `digital_twin.launch.py:265-278` | Passes inline `{"active": False, "input_cloud_topic": cloud_topic}` only -- no config file |
| `mock.launch.py:76-82` | Passes entire config file -- OK |

### 1.5 Prioritized Risks

1. **Launch files bypassing config** (HIGH) -- `grasp_test.launch.py`, `digital_twin.launch.py`, and `twist_propagation_test.launch.py` pass only a few inline parameters, ignoring the YAML config entirely. This means any tuning in `twist_propagation.yaml` or `prosthesis_config.yaml` has no effect for those launch scenarios.

2. **Hardcoded visualization values** (MEDIUM) -- The EMA alpha, marker sizes, and colors are not configurable, making it impossible to tune visualization without code changes.

3. **Hardcoded topic names for publishers** (LOW-MEDIUM) -- 6 publisher topic strings are hardcoded rather than declared as parameters, reducing reusability.

---

## 2. Implementation Plan

### Phase 1: Add Missing Configurable Parameters to the Node

- [ ] **Task 1.1**: Add new ROS parameters for currently hardcoded visualization values in `__init__()`:
  - `twist_ema_alpha` (float, default `0.4`) -- EMA smoothing factor for twist estimation
  - `hit_marker_scale_m` (float, default `0.06`) -- hit marker sphere diameter in metres
  - `hit_marker_lifetime_s` (float, default `2.0`) -- hit marker persistence time in seconds
  - `trajectory_line_width_m` (float, default `0.01`) -- trajectory line width in metres
  - `max_collision_spheres` (int, default `50`) -- max number of collision spheres for visualization
  - `tf_transform_timeout_s` (float, default `0.5`) -- TF2 transform lookup timeout
  - Rationale: These values are currently magic numbers that require code changes to tune. Making them parameters allows runtime/dataset-specific tuning.

- [ ] **Task 1.2**: Read the new parameters into instance variables in the parameter-reading section (after line 400):
  - Store as `self._ema_alpha`, `self._hit_marker_scale`, `self._hit_marker_lifetime`, `self._trajectory_line_width`, `self._max_collision_spheres`, `self._tf_timeout`
  - Rationale: Consistent with the existing pattern where all parameters are read into `self._*` variables.

- [ ] **Task 1.3**: Replace hardcoded literals in the code body with the new instance variables:
  - Line 640: `alpha = 0.4` → `alpha = self._ema_alpha`
  - Line 833: `step = max(1, len(positions) // 50)` → use `self._max_collision_spheres`
  - Line 867: `0.06` → `self._hit_marker_scale`
  - Line 869: `m.lifetime.sec = 2` → compute from `self._hit_marker_lifetime`
  - Line 891: `0.01` → `self._trajectory_line_width`
  - Line 772: `timeout=rclpy.duration.Duration(seconds=0.5)` → use `self._tf_timeout`
  - Rationale: Eliminates all magic numbers from the node logic.

- [ ] **Task 1.4**: Add publisher topic name parameters for the 6 hardcoded visualization topics:
  - `status_topic` (default `"/twist_propagation/status"`)
  - `predicted_path_topic` (default `"/twist_propagation/predicted_path"`)
  - `current_pose_topic` (default `"/twist_propagation/current_pose"`)
  - `collision_spheres_topic` (default `"/twist_propagation/collision_spheres"`)
  - `hit_marker_topic` (default `"/twist_propagation/hit_marker"`)
  - `trajectory_line_topic` (default `"/twist_propagation/trajectory_line"`)
  - Rationale: All other topic names are already configurable; these 6 are the exception. Making them configurable is consistent with the existing design pattern.

- [ ] **Task 1.5**: Update the publisher constructors (lines 472-484) to use the new topic parameters instead of hardcoded strings.
  - Rationale: Completes the config-driven design pattern for all topics.

### Phase 2: Update Configuration Files

- [ ] **Task 2.1**: Add the new parameters to `src/twist_propagation/config/twist_propagation.yaml` under the `twist_propagation.ros__parameters` section:
  - Add a `# ── Visualization ────────────────────────────────────────────────` section with all new visualization parameters
  - Add the new topic parameters to the existing `# ── Topic names` section
  - Rationale: This is the canonical parameter file for the package.

- [ ] **Task 2.2**: Add the same new parameters to `config/prosthesis_config.yaml` in BOTH:
  - The `twist_propagation.ros__parameters` section (lines 96-120) -- used by launch files
  - The flat `twist_propagation:` section (lines 244-266) -- used by test scripts
  - Rationale: The file header explicitly states "When adding new parameters, add them to BOTH the node-namespaced section AND the corresponding flat section."

### Phase 3: Fix Launch Files to Use Config Files

- [ ] **Task 3.1**: Update `grasp_test.launch.py` (lines 110-123) to load parameters from the config file and overlay only the overrides:
  - Load `prosthesis_config.yaml` (or accept via `config_file` argument) as the base parameters
  - Overlay `active` and `input_cloud_topic` on top of the config
  - Rationale: Currently this launch ignores all config tuning (collision thresholds, timeouts, etc.)

- [ ] **Task 3.2**: Update `digital_twin.launch.py` (lines 265-278) with the same pattern:
  - Load config file as base, overlay `active` and `input_cloud_topic`
  - Rationale: Same issue as grasp_test.launch.py.

- [ ] **Task 3.3**: Update `twist_propagation_test.launch.py` (lines 93-105) with the same pattern:
  - Load the package-local `twist_propagation.yaml` as base, overlay `active`, `input_cloud_topic`, and `odom_topic`
  - Rationale: Currently uses only `declare_parameter()` defaults, not the YAML config.

- [ ] **Task 3.4**: Update `pipeline.launch.py` (lines 119-125) to also pass the `active` parameter from config (it already passes the config file, so this may already work, but verify that the `active` key from the YAML is correctly forwarded).
  - Rationale: Ensure consistency; this launch file already does the right thing for most params.

### Phase 4: Update Tests

- [ ] **Task 4.1**: Update `src/twist_propagation/test/test_twist_propagation.py` if any pure-function signatures change (they should not, since all new parameters are node-level).
  - Verify existing tests still pass after changes.
  - Rationale: The pure helper functions are not affected, but we should confirm.

- [ ] **Task 4.2**: Update `scripts/test_twist_propagation_integration.py` to verify the new parameter-driven behavior:
  - Add a test that verifies the node starts inactive by default and can be activated via service
  - Add a test that verifies deactivation stops all propagation
  - The existing tests already cover activation/deactivation (lines 296-400), so this is primarily a verification task
  - Rationale: Ensure the start/stop flag works correctly end-to-end.

- [ ] **Task 4.3**: Add a unit test for the EMA alpha parameter:
  - Verify that `_estimate_twist` uses `self._ema_alpha` correctly (this requires a minor refactor to pass alpha as an argument or make it testable)
  - Rationale: The EMA alpha was a magic number; now that it's configurable, we should verify it works.

### Phase 5: Documentation & Cleanup

- [ ] **Task 5.1**: Update the node's docstring (lines 1-49) to document:
  - The new parameters in the parameter list
  - Clarify the start/stop behavior (parameter `active` for initial state, services for runtime toggling)
  - Rationale: The docstring is the primary API reference for the node.

- [ ] **Task 5.2**: Update the launch file docstrings where parameters were changed.
  - Rationale: Keep documentation in sync with code.

---

## 3. Verification Criteria

- [ ] All hardcoded numeric literals in the node are replaced with configurable parameters
- [ ] All publisher topic names are configurable via parameters
- [ ] All 3 config files (`twist_propagation.yaml`, `prosthesis_config.yaml` both sections) contain the new parameters
- [ ] All launch files that launch the twist propagation node load parameters from a config file (not just inline dicts)
- [ ] Existing unit tests pass without modification
- [ ] Existing integration tests pass
- [ ] The node can be started inactive (`active: false`) and activated at runtime via service call
- [ ] The node can be deactivated at runtime, stopping all propagation and clearing markers
- [ ] No regression in the existing activate/deactivate service behavior

## 4. Potential Risks and Mitigations

1. **Launch file parameter merging complexity**
   Risk: When overlaying launch arguments on top of YAML config, the merging logic must be correct (dict update order matters).
   Mitigation: Follow the pattern already used in `twist_propagation.launch.py` (lines 34-46) which loads YAML then overlays with `params[key] = LaunchConfiguration(...)`.

2. **Config file duplication drift**
   Risk: Parameters exist in 3 places (`twist_propagation.yaml`, `prosthesis_config.yaml` node section, `prosthesis_config.yaml` flat section) and could diverge.
   Mitigation: The `prosthesis_config.yaml` header already warns about this. Consider adding a validation script in the future that checks consistency.

3. **Breaking existing launch file behavior**
   Risk: Changing how launch files pass parameters could break existing workflows (e.g., users who rely on defaults).
   Mitigation: All new parameters have the same defaults as the current hardcoded values, so behavior is identical if no config is provided.

4. **EMA alpha refactoring testability**
   Risk: Making `_estimate_twist` use a parameter might require refactoring the method signature or accessing `self._ema_alpha` which is harder to unit test.
   Mitigation: The method is on the class and already accesses `self._twist`, so accessing `self._ema_alpha` is consistent. No signature change needed.

## 5. Alternative Approaches

1. **Use ROS 2 parameter events for start/stop instead of services**: Could use `OnSetParametersCallback` to react to `active` parameter changes at runtime. However, the existing service-based approach is more explicit, better documented, and already implemented. **Not recommended** -- services are the right pattern here.

2. **Single shared config file**: Instead of having both `twist_propagation.yaml` and `prosthesis_config.yaml`, consolidate into one. This would eliminate duplication but would require updating all launch files and might not align with the project's architecture where `prosthesis_config.yaml` is the central config. **Not recommended** for this change -- too disruptive.

3. **Dynamic parameter declaration**: Use `declare_parameter` with `dynamic_typing=True` for the new visualization parameters. This would allow changing them at runtime via `ros2 param set`. **Recommended for visualization parameters** -- adds flexibility without complexity.
