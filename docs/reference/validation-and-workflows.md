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
