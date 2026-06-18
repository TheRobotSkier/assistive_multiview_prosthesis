# Mia Haptic Force Test Multi-Node Audit Context

Date: 2026-06-18

This document captures common context for the bead set created from the audit of
the split-node rewrite of `scripts/mia_haptic_force_test.py`.

## Scope

The split-node rewrite lives under `scripts/mia_haptic_force_test/` and is
selected through `USE_MULTI_NODE=true` in the option 4 haptic force test path:

- `scripts/mia_haptic_force_test/emg_input_node.py`
- `scripts/mia_haptic_force_test/force_input_node.py`
- `scripts/mia_haptic_force_test/hand_controller_node.py`
- `scripts/mia_haptic_force_test/supervisor_node.py`
- `scripts/mia_haptic_force_test/haptic_node.py`
- `scripts/mia_haptic_force_test/logger_node.py`
- `scripts/mia_haptic_force_test/terminal_ui_node.py`
- `scripts/mia_haptic_force_test/common/README.md`
- `scripts/mia_haptic_force_test/common/constants.py`
- `scripts/mia_haptic_force_test/common/conversions.py`
- `src/prosthesis_launch/launch/mia_haptic_force_test.launch.py`
- `scripts/mia_haptic_force_test.sh`
- `scripts/menu_test.sh`
- `Makefile`
- `config/mia_haptic_force_test.yaml`

Legacy reference:

- `scripts/mia_haptic_force_test.py`

## Hard Requirements

The option 4 haptic force test is PC-only. It must not depend on cameras,
Jetson, segmentation, pointcloud fusion, preshaping, or the production pipeline.
Allowed hardware is:

- Mia Hand
- optional wrist Dynamixel
- optional Vibro8 haptic band
- EMG board

The legacy monolithic script must remain runnable with `USE_MULTI_NODE=false`.
Do not remove or break that path while repairing the split-node path.

## Current Audit Verdict

Do not use `USE_MULTI_NODE=true` on hardware yet. The split-node rewrite is not
parity-equivalent with the legacy test and has multiple safety-critical gaps.
The most important issues are:

- real EMG input is broken or silently falls back to simulated REST
- controller output is not actually gated by `/control/enable`
- force hold publishes `mode="hold"` but the hand controller has no hold branch
- open/release uses velocity control with open positions treated as force targets
- controller-manager calls can reintroduce executor recursion/blocking in the
  control loop
- no implemented node publishes `/hand/joint_states`
- the split launch has no clean completion/shutdown path
- EMG confidence/freshness/hold/proportional gating was dropped
- wrist stage completion is based on internal target instead of actual wrist
  state
- force stale, required-force, max-closure, and emergency fault semantics were
  dropped or weakened
- `/control/target_wrist` has a message type mismatch in the contract/logger
- CSV output is not legacy-compatible and likely writes outside the host-mounted
  `/app/data` location
- `config_path` and many parameter-editor values are ignored by split nodes
- `test-grasp` wrist detection/passing is broken
- haptic hold-mode behavior is not parity-equivalent

## Useful Commands

Static checks:

```bash
python3 -m py_compile \
  scripts/mia_haptic_force_test/*.py \
  scripts/mia_haptic_force_test/common/*.py \
  scripts/mia_haptic_force_test/launch/*.py \
  src/prosthesis_launch/launch/mia_haptic_force_test.launch.py \
  scripts/mia_haptic_force_test.py

PYTHONPATH=src/force_controller \
  python3 -m pytest src/force_controller/test/test_controller_manager_client.py -q
```

Hardware/ROS verification should be bounded:

```bash
AUTO_KILL_S=20 FORCE_RETRAIN=false USE_MULTI_NODE=true make test-grasp
make test-grasp-down
```

Use `USE_MULTI_NODE=false` as the known legacy fallback:

```bash
AUTO_KILL_S=20 FORCE_RETRAIN=false USE_MULTI_NODE=false make test-grasp
make test-grasp-down
```

Do not run multiple hardware/container option 4 tests in parallel.

## Evidence From Audit

Static checks passed:

- split and legacy scripts compiled with `python3 -m py_compile`
- `src/force_controller/test/test_controller_manager_client.py` passed
  (`20 passed`)

The split timing/integration scripts could not be run in the host Python or the
existing `mia-haptic-force-test` container because `rclpy` was not importable in
that environment. Do not claim timing compliance until the tests run in an
environment with ROS 2 Python available.

## Safety Notes

Treat the split-node path as unsafe until the P0 beads are resolved and verified.
The legacy monolithic path contains safety behavior that the split stack must
preserve before hardware use:

- explicit controller readiness and active-controller verification
- open hand on startup and release/fault paths
- EMG freshness, confidence, hold-time, and proportional threshold checks
- force-data stale/unavailable faults
- emergency force backoff/fault
- max-closure before contact fault
- wrist completion using actual wrist feedback or configured timeout
- zero haptics on complete/fault/shutdown

