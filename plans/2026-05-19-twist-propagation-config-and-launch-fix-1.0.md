# Fix Twist Propagation Config Consistency & Launch File Loading

## Objective

Fix two concrete problems:
1. **Config values disagree with node defaults** -- `collision_geometry_radius_m` and `cycle_delay_s` have different values in the YAML configs vs. the node's `declare_parameter()` fallbacks. Align them.
2. **Three launch files bypass the config entirely** -- `twist_propagation_test.launch.py`, `grasp_test.launch.py`, and `digital_twin.launch.py` pass only inline parameter dicts, so any tuning in the YAML has no effect. Fix them to load the YAML as a base and overlay only what they need to override.

---

## Problem Detail

### Value discrepancies between node defaults and YAML configs

| Parameter | Node `declare_parameter` default | YAML config value | Files affected |
|---|---|---|---|
| `collision_geometry_radius_m` | **0.05** (`twist_propagation_node.py:350`) | **0.10** (`twist_propagation.yaml:18`, `prosthesis_config.yaml:103,250`) | All configs |
| `cycle_delay_s` | **0.1** (`twist_propagation_node.py:343`) | **0.5** (`prosthesis_config.yaml:98,245`) | `prosthesis_config.yaml` only |

Note: `twist_propagation.yaml:9` already has `cycle_delay_s: 0.1` (matching the node default), but `prosthesis_config.yaml` has `0.5` in both sections.

### Launch files not loading config

| Launch file | What it passes today | Config loaded? |
|---|---|---|
| `twist_propagation.launch.py` | Full YAML + overlay | Yes (correct) |
| `pipeline.launch.py` | Full `prosthesis_config.yaml` | Yes (correct) |
| `mock.launch.py` | Full `prosthesis_config.yaml` | Yes (correct) |
| `twist_propagation_test.launch.py:98-102` | `{active, input_cloud_topic, odom_topic}` only | **No** |
| `grasp_test.launch.py:115-119` | `{active, input_cloud_topic}` only | **No** |
| `digital_twin.launch.py:270-274` | `{active, input_cloud_topic}` only | **No** |

---

## Implementation Plan

### Phase 1: Align Config Values

- [x] **Task 1.1**: Update `collision_geometry_radius_m` in `src/twist_propagation/config/twist_propagation.yaml:18` from `0.10` to `0.05` to match the node default.
  Rationale: User confirmed `0.05` is the preferred value. This is the canonical config for the package.

- [x] **Task 1.2**: Update `collision_geometry_radius_m` in `config/prosthesis_config.yaml` in the `ros__parameters` section (line 103) from `0.10` to `0.05`.
  Rationale: Must match the package-local config.

- [x] **Task 1.3**: Update `collision_geometry_radius_m` in `config/prosthesis_config.yaml` in the flat reference section (line 250) from `0.10` to `0.05`.
  Rationale: The file header requires both sections to stay in sync.

- [x] **Task 1.4**: Update `cycle_delay_s` in `config/prosthesis_config.yaml` in the `ros__parameters` section (line 98) from `0.5` to `0.1`.
  Rationale: Must match the package-local config (`twist_propagation.yaml:9` already has `0.1`) and the node default.

- [x] **Task 1.5**: Update `cycle_delay_s` in `config/prosthesis_config.yaml` in the flat reference section (line 245) from `0.5` to `0.1`.
  Rationale: Keep both sections in sync.

### Phase 2: Fix Launch Files to Load Config

The pattern to follow already exists in `twist_propagation.launch.py:23-57`: load the YAML, extract the `ros__parameters` dict, overlay launch arguments on top, pass the merged dict to the node.

- [x] **Task 2.1**: Fix `twist_propagation_test.launch.py` (lines 92-105) to load the package-local config and overlay overrides:
  - In `_launch_setup()`, load `twist_propagation.yaml` using the same YAML-reading pattern from `twist_propagation.launch.py:34-39`
  - Build a `params` dict from the YAML's `twist_propagation.ros__parameters`
  - Overlay: `params["active"] = active == "true"`, `params["input_cloud_topic"] = input_cloud_topic`, `params["odom_topic"] = "/hand_odom"`
  - Pass `parameters=[params]` to the node
  Rationale: This ensures all tuned parameters (collision thresholds, timeouts, etc.) are used, not just the 3 inline ones.

- [x] **Task 2.2**: Fix `grasp_test.launch.py` (lines 110-123) to load config and overlay overrides:
  - Load `prosthesis_config.yaml` (via the `config_file` launch argument or the default path already defined in the file)
  - Extract `twist_propagation.ros__parameters` from it
  - Overlay: `active: True`, `input_cloud_topic: cloud_topic`
  - Pass merged params to the node
  Rationale: Same issue -- all tuning is currently ignored.

- [x] **Task 2.3**: Fix `digital_twin.launch.py` (lines 265-278) to load config and overlay overrides:
  - Same pattern as Task 2.2: load `prosthesis_config.yaml`, extract twist_propagation params
  - Overlay: `active: False`, `input_cloud_topic: cloud_topic`
  - Pass merged params to the node
  Rationale: Same issue.

### Phase 3: Verify

- [x] **Task 3.1**: Run existing unit tests to confirm nothing broke:
  `python3 -m pytest src/twist_propagation/test/test_twist_propagation.py -v`
  Rationale: Pure function tests should be unaffected, but verify.

- [x] **Task 3.2**: Run integration test to confirm the node still works end-to-end:
  `python3 scripts/test_twist_propagation_integration.py`
  Rationale: The integration test also passes inline params (not via launch), so it should be unaffected, but verify.

- [ ] **Task 3.3**: Manual verification -- launch the node via the fixed `twist_propagation_test.launch.py` and confirm:
  - `ros2 param get /twist_propagation collision_geometry_radius_m` returns `0.05`
  - `ros2 param get /twist_propagation cycle_delay_s` returns `0.1`
  - Node behavior is unchanged from before (since the YAML now matches the node defaults you were already getting)
  Rationale: Confirms the config loading works correctly.

---

## Verification Criteria

- All 3 config files have `collision_geometry_radius_m: 0.05` and `cycle_delay_s: 0.1`
- All 3 fixed launch files load the YAML config as a base before overlaying their overrides
- `ros2 param list /twist_propagation` shows all expected parameters when launched via any of the fixed launch files
- Existing tests pass without modification

## Potential Risks and Mitigations

1. **YAML loading path resolution in launch files**
   Risk: The config file path must be resolved correctly in each launch file's context (installed vs. source).
   Mitigation: `twist_propagation_test.launch.py` can use `get_package_share_directory("twist_propagation")` to find the installed config. `grasp_test.launch.py` and `digital_twin.launch.py` already have `config_file` arguments and `_WORKSPACE_ROOT` patterns for finding `prosthesis_config.yaml`.

2. **Behavior change for `prosthesis_config.yaml` users**
   Risk: Anyone relying on `cycle_delay_s: 0.5` or `collision_geometry_radius_m: 0.10` from `prosthesis_config.yaml` will see a behavior change.
   Mitigation: The node defaults were always `0.1` and `0.05`, so `pipeline.launch.py` and `mock.launch.py` (which load this config) were already using the YAML values. This change aligns the YAML with the node defaults, which is what the user wants. The `twist_propagation.yaml` already had `cycle_delay_s: 0.1`.
