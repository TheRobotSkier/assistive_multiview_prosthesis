# Interactive Hardware Test Script for Force Controller & Pipeline Manager

## Objective

Create a single interactive Python script (`tests/test2_hw_force_controller/hw_test.py`) that runs inside the container and provides a menu-driven interface for testing the force controller and pipeline manager with the physical Mia Hand. The script assumes the Mia Hand driver is already running and launches the remaining nodes (command_bridge, force_controller, pipeline_manager) itself.

## Implementation Plan

- [ ] Task 1. Create `tests/test2_hw_force_controller/` directory
- [ ] Task 2. Create `tests/test2_hw_force_controller/hw_test.py` — the main interactive test script with the following menu-driven routines:
  - **Pre-flight check**: Verify `/driver` node is running, `/dev/ttyUSB0` exists, and force/joint data streams are available. Abort with clear instructions if not.
  - **Auto-launch**: Start command_bridge, force_controller, and pipeline_manager as subprocesses. Wait for each to report ready by polling `ros2 node list`.
  - **Interactive menu** with these options:
    1. **Finger Jog** — Move individual fingers to a target position (user inputs finger name + angle in rad). Publishes to `*_pos_ff_controller/commands`.
    2. **Open Hand** — Publish 0.0 to all three finger command topics.
    3. **Close Hand** — Publish user-specified closure amount (default 1.5 rad) to all three.
    4. **Force Monitor** — Subscribe to `data_streams/fingers/forces/data` and `/force_controller/status` for N seconds, print a live table of forces, errors, stability, and slip status. Summarize min/max/mean at the end.
    5. **Activate Force Controller** — Inject APPROACHING(3) then GRASPING(4) on `/pipeline/state`, then run the force monitor automatically to show the PI loop regulating.
    6. **Release** — Inject RELEASING(6) then IDLE(0) on `/pipeline/state`.
    7. **Pipeline State Injection** — Manually inject any pipeline state (0-6) by number.
    8. **Full Grasp Routine** — Orchestrated sequence: close to partial → inject APPROACHING → inject GRASPING → monitor force regulation for user-specified seconds → inject RELEASING → open hand. Print a summary with force trajectories.
    9. **Calibration Helper** — Read and display current joint positions and raw force readings for 5 seconds so the user can note baselines (open hand, touching object, etc.).
    0. **Quit** — Kill all launched subprocesses and exit.
  - **Logging**: All force data and pipeline events during routines are written to `tests/test2_hw_force_controller/results/` as timestamped CSV files.
  - **Safety**: Emergency release on any error. Max force check during all routines (abort and back off if forces exceed emergency threshold).

- [ ] Task 3. Verify the script works by running it inside the container (dry-run of menu display, pre-flight checks).

## Verification Criteria

- Script starts, detects the running driver, and auto-launches the 3 remaining nodes
- All menu options execute without errors
- Force data is displayed in real-time during monitoring
- CSV results are written with correct timestamps and force values
- Emergency release works when forces exceed threshold
- Clean shutdown kills all subprocesses

## Potential Risks and Mitigations

1. **Subprocess management in container** — Python subprocesses may not clean up properly on Ctrl-C
   Mitigation: Use `atexit` and signal handlers to kill all child processes
2. **ROS2 node name conflicts** — Running test node alongside existing nodes
   Mitigation: Use unique node name `hw_test_runner` with random suffix
3. **Force data stale during testing** — If force streaming isn't activated
   Mitigation: Pre-flight check activates force streaming; force controller also activates it on GRASPING

## Alternative Approaches

1. **Separate launch + test**: Could split into a launcher script and a test script, but a single script is simpler for interactive use
2. **ROS2 launch file**: Could use a launch file to start nodes, but interactive control via topics is cleaner in a single script
