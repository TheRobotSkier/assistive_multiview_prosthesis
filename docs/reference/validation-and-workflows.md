# Validation And Workflows

## Main Build Paths

### Host-level

File: `Makefile`

Common commands:

- `make dev`: start the prosthesis container
- `make shell`: shell into the running container
- `make emg-dev`: start the dedicated EMG container
- `make emg-shell`: shell into the dedicated EMG container
- `make build-prosthesis`: build main runtime container
- `make segmentation-cpu` or `make segmentation-cuda`: start segmentation service
- `make mia-haptic-force-test`: run the isolated Mia/EMG/wrist/haptic force test container
- `make test`: run containerized test target
- `make emg-latency-workflow`: run EMG collection, training, and interactive latency benchmarking
- `make emg-latency-workflow-notrain`: skip collection/training, reuse existing model
- `make emg-simulate`: offline prediction simulator — replay raw data with tunable parameters

### In-container

File: `Makefile.workspace`

Common commands:

- `make build`: build all ROS packages with `colcon`
- `make build-pkg PKG=<package>`: build one package
- `make test`: run `scripts/run_tests.sh`
- `make pipeline`: launch the full hardware pipeline
- `make run`: build then launch the hardware pipeline
- `make run-emg-grasp`: build then launch the simplified EMG pipeline
- `make run-mia-haptic-force-test`: build then launch the Mia haptic force test from inside its container
- `make camera-test`: build selected packages and run a hardware-light camera validation path
- `ros2 run emg_bridge collect_data`: interactive EMG recording
- `ros2 run emg_bridge train`: offline EMG model training
- `ros2 run emg_bridge run_classifier`: live EMG inference display
- `ros2 run emg_bridge latency_benchmark`: interactive latency benchmark with CSV export
- `ros2 run emg_bridge prediction_simulator`: offline simulator with parameter-tuning menu

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
- `scripts/mia_haptic_force_test.py`
- `scripts/grasp_test.sh`
- `src/emg_bridge/test/test_latency_analysis.py`
- `src/emg_bridge/test/test_latency_benchmark_cli.py`

## EMG Workflow

Primary host entrypoint:

- `scripts/emg_latency_workflow.sh`
- `make emg-latency-workflow`

Default flow:

1. Use the current local checkout on `asger_dev`.
2. Ensure the dedicated EMG container is running.
3. Build `emg_bridge` in-container.
4. Run `collect_data`.
5. Run `train`.
6. Run `latency_benchmark` interactively.
7. Optionally commit and push generated `data/` and `models/` artifacts on `asger_dev` only.

Container details:

- The EMG workflow now runs in a dedicated `emg` compose service instead of the general `prosthesis` dev container.
- The dedicated EMG image is built from `ros:jazzy-ros-core-noble` to avoid pulling the larger desktop stack for EMG-only work.
- The slim image keeps `collect_data`, `train`, `run_classifier`, `latency_benchmark`, and ROS topic publishing support for `/emg/*` topics.
- The workflow does not fetch from the remote repository by default; set `EMG_FETCH_REMOTE=true` only if you explicitly want a pre-run `git fetch origin asger_dev`.

Key environment variables:

- `EMG_COLLECT_REPS`
- `EMG_COLLECT_DURATION_S`
- `EMG_LATENCY_REPEATS`
- `EMG_LATENCY_BASELINE_S`
- `EMG_LATENCY_TAIL_S`
- `EMG_LATENCY_RESULT_DIR_NAME`
- `EMG_PUSH_RESULTS`

Latency benchmark notes:

- `ros2 run emg_bridge latency_benchmark` writes sample-level EMG, frame-level predictions, per-trial latency summaries, aggregate latency stats, and run metadata to CSV/JSON under the chosen output directory.
- The onset detector now links the accepted prediction to the same EMG signal across the last `N` target-predicting windows instead of only searching immediately before the final support window.
- Use `--onset-lookback-windows` to control how many recent target-predicting windows must share consistent per-channel activation. The default matches the classifier smoothing window.
- `--pre-onset-search-samples` still extends the search slightly earlier than the earliest qualifying window when needed, but it no longer has to carry the full multi-window lookback by itself.

## Mia Haptic Force Test Workflow

Primary host entrypoint:

- `make mia-haptic-force-test`

Default flow:

1. Detect Mia Hand and wrist USB serial ports with `scripts/detect_usb_host.sh`.
2. Start the dedicated compose service `mia-haptic-force-test` with its own build/install/log volumes. The service bind-mounts `/dev` instead of fixed USB device paths so it can start even when the Mia Hand or wrist are unplugged.
3. Build the ROS packages needed for `mia_hand_ros2_control`, EMG inference, wrist control, haptics, and launch composition.
4. Reuse or train an EMG classifier in `/app/models` from recordings in `/app/data`.
5. Launch `src/prosthesis_launch/launch/mia_haptic_force_test.launch.py`.
6. Run the staged test until the user holds OPEN for the configured stop duration.
7. Write CSV outputs under `data/mia_haptic_force_test/<run-id>/` on the host.

Important files:

- Config: `config/mia_haptic_force_test.yaml`
- Test node and CSV logger: `scripts/mia_haptic_force_test.py`
- Wrapper: `scripts/mia_haptic_force_test.sh`
- Launch: `src/prosthesis_launch/launch/mia_haptic_force_test.launch.py`
- Container service: `docker/docker-compose.yml` service `mia-haptic-force-test`

Generated artifacts:

- `samples.csv`: state, EMG prediction, wrist state, hand joint state, force values, force targets, commands, haptic motor percentages, and optional raw Mia streams
- `events.csv`: state transitions, haptic buzz events, mode toggles, faults, and shutdown
- `config_snapshot.yaml`: copy of the YAML configuration used for the run

Hardware absence behavior:

- If the Mia Hand serial device is missing, the host target defaults `MOCK_HARDWARE=true`.
- If the wrist serial device is missing, the host target defaults `WRIST_ENABLE=false`.
- If the Vibro8 haptics are absent, run with `HAPTIC_ENABLE=false` to avoid Bluetooth reconnect noise.
- `MOCK_HARDWARE=true` only mocks the Mia Hand; live EMG still runs when `EMG_ENABLE=true`.

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
