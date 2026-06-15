# Validation And Workflows

## Main Build Paths

### Host-level

File: `Makefile`

Common commands:

- `make dev`: start the prosthesis container
- `make shell`: shell into the running container
- `make build-prosthesis`: build main runtime container
- `make segmentation-cpu` or `make segmentation-cuda`: start segmentation service
- `make test`: run containerized test target
- `make emg-latency-workflow`: run EMG collection, training, and interactive latency benchmarking

### In-container

File: `Makefile.workspace`

Common commands:

- `make build`: build all ROS packages with `colcon`
- `make build-pkg PKG=<package>`: build one package
- `make test`: run `scripts/run_tests.sh`
- `make pipeline`: launch the full hardware pipeline
- `make run`: build then launch the hardware pipeline
- `make run-emg-grasp`: build then launch the simplified EMG pipeline
- `make camera-test`: build selected packages and run a hardware-light camera validation path
- `ros2 run emg_bridge collect_data`: interactive EMG recording
- `ros2 run emg_bridge train`: offline EMG model training
- `ros2 run emg_bridge run_classifier`: live EMG inference display
- `ros2 run emg_bridge latency_benchmark`: interactive latency benchmark with CSV export

## Smoke Test Entry Point

Main script: `scripts/run_tests.sh`

It runs these tests:

- `scripts/test_build.sh`
- `scripts/test_launch_syntax.sh`
- `scripts/test_preshaping_so.sh`
- `scripts/test_nodes_start.sh`
- `scripts/test_twist_propagation.sh`

## Other Important Test Scripts

- `scripts/test_pointcloud_health.sh`
- `scripts/test_twist_propagation_integration.py`
- `scripts/visual_smoke_test_twist_propagation.py`
- `scripts/static_grasp_test.sh`
- `scripts/grasp_test.sh`
- `src/emg_bridge/test/test_latency_analysis.py`
- `src/emg_bridge/test/test_latency_benchmark_cli.py`

## EMG Workflow

Primary host entrypoint:

- `scripts/emg_latency_workflow.sh`
- `make emg-latency-workflow`

Default flow:

1. Fetch `origin/asger_dev`.
2. Ensure the `prosthesis` dev container is running.
3. Build `emg_bridge` in-container.
4. Run `collect_data`.
5. Run `train`.
6. Run `latency_benchmark` interactively.
7. Optionally commit and push generated `data/` and `models/` artifacts on `asger_dev` only.

Key environment variables:

- `EMG_COLLECT_REPS`
- `EMG_COLLECT_DURATION_S`
- `EMG_LATENCY_REPEATS`
- `EMG_LATENCY_BASELINE_S`
- `EMG_LATENCY_TAIL_S`
- `EMG_LATENCY_RESULT_DIR_NAME`
- `EMG_PUSH_RESULTS`

## Safe Validation Heuristic For Agents

When making code changes:

1. Run the smallest relevant build/test first.
2. Prefer package-local validation before full workspace validation.
3. If launch composition changed, validate launch syntax.
4. If runtime parameters changed, verify `config/prosthesis_config.yaml` still matches node declarations.
5. If topics/services/frames changed, update docs in the same change.

## Where Verification Usually Lives

- Build and launch orchestration: `Makefile`, `Makefile.workspace`
- Shell-based smoke tests: `scripts/`
- Package-local tests: within package directories, for example `src/twist_propagation/test/` and `src/pointcloud_fusion/test/`
- Scenario verification assets: `tests/`
