# Agent Instructions

Use this file first when working in this repository.

## Purpose

This repo is a ROS 2 Jazzy workspace for a multiview prosthesis pipeline spanning perception, segmentation, grasp planning, orchestration, hand/wrist actuation, EMG input, and haptic feedback.

## Start Here

Before changing code, read:

- `docs/reference/directory-structure.md`
- `docs/architecture/overview.md`

Then open the subsystem-specific document that matches the task:

- runtime composition: `docs/architecture/runtime-entrypoints.md`
- perception, TF, fusion, segmentation, twist targeting: `docs/architecture/perception.md`
- state machine, planning, force control, actuation, haptics: `docs/architecture/control-and-actuation.md`
- package ownership: `docs/architecture/packages.md`
- validation entry points: `docs/reference/validation-and-workflows.md`

## Fast Navigation Map

Top-level reference:

- `config/`: runtime configuration
- `docker/`: container definitions
- `docs/`: canonical documentation
- `scripts/`: test/setup/utilities
- `src/`: ROS packages
- `tests/`: higher-level verification assets
- `Makefile`: host workflow
- `Makefile.workspace`: in-container workflow

High-value code entry points:

- full pipeline launch: `src/prosthesis_launch/launch/pipeline.launch.py`
- simplified EMG pipeline: `src/prosthesis_launch/launch/simple_emg_grasp.launch.py`
- main state machine: `src/pipeline_manager/pipeline_manager/pipeline_manager_node.py`
- central config: `config/prosthesis_config.yaml`
- fusion: `src/pointcloud_fusion/pointcloud_fusion/pointcloud_fusion_node.py`
- twist targeting: `src/twist_propagation/twist_propagation/twist_propagation_node.py`
- grasp planning bridge: `src/grasp_preshaping/nodes/preshaping_service_bridge_node.cpp`
- approach controller: `src/grasp_preshaping/nodes/grasp_proximity_controller_node.py`
- hand command adapter: `src/command_bridge/command_bridge/command_bridge_node.py`
- force controller: `src/force_controller/force_controller/force_controller_node.py`
- EMG latency workflow: `scripts/emg_latency_workflow.sh`
- EMG latency runner: `src/emg_bridge/emg_bridge/scripts/latency_benchmark.py`
- EMG latency analysis: `src/emg_bridge/emg_bridge/latency_analysis.py`

Use `docs/reference/directory-structure.md` for the full map.

## Working Rules For Agents

- Verify architecture facts from code, not from stale memory.
- Start from launch files and config when trying to understand runtime behavior.
- Treat `config/prosthesis_config.yaml` as a critical integration file.
- Be careful around cross-package contracts: topics, services, TF frames, and controller ownership.
- For EMG work, prefer reusing the existing `collect_data`, `train`, `run_classifier`, and `latency_benchmark` entry points instead of adding parallel workflows.
- For EMG latency work, keep onset logic tied to the same recent target-predicting window history that supports the accepted classifier output, including per-channel consistency when looking back across multiple windows.
- For EMG container/runtime work, prefer the dedicated `emg` compose service and `docker/Dockerfile.emg` rather than expanding the general `prosthesis` dev container unless the change truly affects the whole workspace.
- If changing vendored dependencies in `src/open_vins/` or `src/realsense-ros/`, confirm that the task really requires it.

## Required Documentation Update Rule

If you change architecture, runtime flow, package ownership, important file locations, config structure, build/test workflow, or cross-package interfaces, you must update the documentation in the same change.

## Required Documentation Update Procedure

Follow these steps every time a relevant change is made.

1. Identify the changed subsystem using `docs/reference/directory-structure.md`.
2. Re-read the actual source files involved so the docs reflect code, not assumptions.
3. Update the most relevant architecture file in `docs/architecture/`.
4. Update `docs/reference/directory-structure.md` if navigation paths, ownership, or lookup guidance changed.
5. Update `docs/reference/validation-and-workflows.md` if build, test, or run workflows changed.
6. Update `agents.md` if future agents need different navigation or working guidance.
7. Remove stale statements rather than leaving contradictory notes behind.
8. Verify that all referenced paths, launch files, package names, topics, services, and frames still match the code.
9. Check the final diff and confirm docs changed alongside code.

For more detail, follow:

- `docs/reference/documentation-maintenance.md`

## Minimum Doc Files To Check Before Finishing

- `docs/architecture/overview.md`
- the relevant subsystem doc under `docs/architecture/`
- `docs/reference/directory-structure.md`
- `docs/reference/documentation-maintenance.md`
- `agents.md`
