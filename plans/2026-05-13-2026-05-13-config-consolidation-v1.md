# Config Consolidation Plan

## Objective

Fix all hardcoded values in the pipeline nodes so that tuning parameters and topic names are read from the central `prosthesis_config.yaml` instead of being duplicated in Python source code. This eliminates the two active bugs (integral_limit and stale_data_timeout mismatches) and makes the config file the single source of truth.

---

## Current State Analysis

### How Config Flows Today

1. Launch files pass `config_file` as a string parameter to nodes
2. Nodes receive it via `declare_parameter("config_file", "")` but **no node reads the YAML**
3. Each node uses `declare_parameter` with hardcoded defaults — the config YAML values are never loaded
4. The config file is effectively documentation-only

### The Proper ROS2 Pattern (already used by twist_propagation_node)

- Declare each parameter with `declare_parameter("name", default_value)`
- Read it with `get_parameter("name").value`
- The launch file can override defaults by passing a parameters dict or YAML file path

### Bugs Found

| Bug | Node | Hardcoded | Config Says | Impact |
|-----|------|-----------|-------------|--------|
| integral_limit | force_controller_node.py:121 | `5.0` | `50.0` | Integral term clamps too aggressively — controller may not converge |
| stale_data_timeout | force_controller_node.py:359 | `2.0` | `0.5` | Stale data detected 4x slower than config intends |

---

## Implementation Plan (STATUS: ALL COMPLETE)

### Phase 1: Fix the Two Active Bugs in Force Controller

- [x] **Task 1.1** DONE — `integral_limit` declared as parameter (default 50.0), hardcoded `5.0` removed

- [x] **Task 1.2** DONE — `stale_data_timeout_s` declared as parameter (default 0.5), hardcoded `2.0` removed

- [x] **Task 1.3** DONE — `emergency_backoff_factor` declared as parameter (default 2.0), hardcoded `2.0` removed

- [x] **Task 1.4** DONE — Docstring updated with all new parameters

### Phase 2: Make Force Controller Read Topic Names from Parameters

- [x] **Task 2.1** DONE — 8 topic/service parameters declared in force_controller_node.py
- [x] **Task 2.2** DONE — All hardcoded topic strings replaced with parameter values

### Phase 3: Make Pipeline Manager Read Topic Names from Parameters

- [x] **Task 3.1** DONE — 7 topic/service parameters declared in pipeline_manager_node.py
- [x] **Task 3.2** DONE — All hardcoded topic strings replaced with parameter values

### Phase 4: Make Proximity Controller Read Topic Names from Parameters

- [x] **Task 4.1** DONE — 9 topic parameters declared in proximity_controller_node.py
- [x] **Task 4.2** DONE — All hardcoded topic strings replaced with parameter values

### Phase 5: Update Config YAML and Launch Files

- [x] **Task 5.1** DONE — Added `pipeline_state_name`, `force_controller_status`, `joint_states` to topics section

- [x] **Task 5.2** DONE — Launch files now pass YAML path directly as `parameters=[LaunchConfiguration("config_file")]` (ROS2 loads it as parameter file)

- [x] **Task 5.3** DONE — Config restructured with `force_controller:`, `pipeline_manager:`, `proximity_controller:`, `twist_propagation:` node-namespaced `ros__parameters:` sections. Flat reference sections preserved for backward compatibility.

- [x] **Task 5.4** DONE — Both launch files updated: default config path resolves to workspace `config/prosthesis_config.yaml`, YAML passed directly as parameter file to all nodes

---

## Verification Criteria

- [ ] `force_controller_node.py` reads `integral_limit` from parameter (not hardcoded `5.0`)
- [ ] `force_controller_node.py` reads `stale_data_timeout_s` from parameter (not hardcoded `2.0`)
- [ ] `force_controller_node.py` reads all topic names from parameters
- [ ] `pipeline_manager_node.py` reads all topic names from parameters
- [ ] `grasp_proximity_controller_node.py` reads all topic names from parameters
- [ ] `prosthesis_config.yaml` uses ROS2-compatible node-namespaced structure
- [ ] Both launch files pass the YAML as a parameter file
- [ ] All nodes still work with defaults if no config file is provided (backward compatible)
- [ ] No hardcoded topic strings remain in any of the three nodes

---

## Potential Risks and Mitigations

1. **Config YAML restructuring breaks other consumers**
   - The existing flat `topics:`, `hardware:`, `frames:` sections might be read by other tools (e.g., test scripts at `tests/test1_software_verification/run.py:618` which does `yaml.safe_load`).
   - Mitigation: Keep the existing flat sections as-is and add the new namespaced sections alongside them. Both formats coexist. Remove the flat sections only after verifying nothing else reads them.

2. **Launch file path resolution**
   - The config file path is a launch argument with empty default. If empty, the YAML won't be found.
   - Mitigation: Set the default to the workspace-relative path `config/prosthesis_config.yaml` using `os.path.join` in the launch file, so it works without specifying the argument.

3. **Parameter name mismatches between YAML and declare_parameter**
   - If the YAML key doesn't match the `declare_parameter` name exactly, the parameter won't be loaded.
   - Mitigation: Use exact same names in both places. Test by running with `--log-level debug` and checking parameter values at startup.

4. **Scope creep — twist_propagation already works correctly**
   - The twist propagation node already declares topic parameters correctly but the launch file doesn't pass them from config either.
   - Mitigation: Don't change twist_propagation in this plan. It already follows the right pattern. If the launch file is updated to pass the YAML, twist_propagation will automatically pick up its parameters.

---

## Alternative Approaches

1. **Minimal fix only (just the 2 bugs)**: Only fix `integral_limit` and `stale_data_timeout` as declared parameters. Leave topic names hardcoded. Fastest to implement but leaves the config file as documentation-only for topics.

2. **Node-side YAML loading**: Instead of restructuring the YAML for ROS2, have each node read the `config_file` parameter, load the YAML with `yaml.safe_load`, and extract its own values. This keeps the current YAML structure but adds YAML parsing to every node. More code per node, but no launch file changes needed.

3. **Full ROS2 parameter file approach (recommended)**: Restructure the YAML to use `node_name: ros__parameters:` namespaces and pass it as a parameter file. This is the standard ROS2 pattern and requires no YAML parsing in nodes — `declare_parameter` + `get_parameter` automatically pick up values from the parameter file. Requires YAML restructuring and launch file changes but is the cleanest long-term solution.
