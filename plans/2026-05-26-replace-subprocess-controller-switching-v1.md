# Replace Subprocess-Based Controller Switching with Native ROS 2 Service Clients

## Objective

Replace all `subprocess.run(["ros2", ...])` calls in the force controller node (`force_controller_node.py`) with native rclpy service clients to the `controller_manager`. This eliminates the fragile pattern of spawning new processes and DDS participants for each controller switch operation, which is the root cause of the timeout failures during GRASPING and RELEASING transitions.

The same fix should also be applied to `scripts/emg_force_grasp_bridge.py` which has an identical copy of the subprocess-based controller switching code.

---

## Investigation Findings

### The Problem is Real and Severe

1. **Six subprocess calls per controller switch**: A single `_switch_controllers()` invocation spawns up to 6 separate `ros2` CLI processes:
   - `_wait_for_controller_manager()` → `ros2 service list` (1-3 attempts with 3s timeout each)
   - `_controller_states()` → `ros2 service call /controller_manager/list_controllers` (1 call)
   - `_ensure_controller_loaded()` → `ros2 service call /controller_manager/load_controller` + `/controller_manager/configure_controller` (0-2 calls)
   - `_switch_controllers()` → `ros2 service call /controller_manager/switch_controller` (1-2 calls with BEST_EFFORT fallback)

2. **Each subprocess spawns a new DDS participant**: The `ros2` CLI tool is a standalone process that must discover all ROS 2 nodes via DDS before it can list services or call them. Under CycloneDDS with `--network host`, this discovery takes 1-5 seconds per invocation under good conditions, and 5-10+ seconds when the Mia Hand driver is cycling.

3. **Called from the rclpy executor thread**: `_activate_controller()` is called from `_on_pipeline_state()` (line 444), which is a subscription callback running on the single-threaded rclpy executor. The `time.sleep(self._relaxed_wait_s)` at line 511 and the blocking subprocess calls block the entire executor — no callbacks, timers, or service responses can be processed during the switch.

4. **Timeout fragility**: The `_wait_for_controller_manager(timeout_s=5)` uses a 3-second per-attempt timeout on `ros2 service list`. When the Mia Hand driver is disconnecting/reconnecting (which happens frequently during serial bus contention), the controller_manager service becomes transiently unavailable, and 3 seconds is not enough for DDS discovery.

5. **String parsing of output**: The code parses `ros2 service call` stdout using regex (`re.findall(r"name='([^']+)'.*?state='([^']+)'", out)`) and string matching (`"ok=True" not in out`). This is brittle and dependent on the exact output format of the `ros2` CLI tool.

### Affected Files

| File | Lines | Role |
|------|-------|------|
| `src/force_controller/force_controller/force_controller_node.py` | 90-201 | Production force controller — subprocess functions at module level |
| `scripts/emg_force_grasp_bridge.py` | 206-317 | EMG bridge — identical subprocess functions at module level |

### The Correct Fix: Native ROS 2 Service Clients

The `controller_manager` exposes standard ROS 2 services:
- `/controller_manager/list_controllers` (`controller_manager_msgs/srv/ListControllers`)
- `/controller_manager/load_controller` (`controller_manager_msgs/srv/LoadController`)
- `/controller_manager/configure_controller` (`controller_manager_msgs/srv/ConfigureController`)
- `/controller_manager/switch_controller` (`controller_manager_msgs/srv/SwitchController`)

These can be called directly via rclpy service clients that live within the same node, sharing the existing DDS participant. This eliminates:
- DDS discovery overhead (the node already has a participant)
- Process spawning latency (no fork/exec)
- Timeout fragility (service calls use the node's existing DDS transport)
- Blocking sleeps in the callback thread (can use `call_async()` + future callbacks)

---

## Implementation Plan

### Phase 1: Create a Shared Controller Manager Client Module

- [ ] **Task 1.1.** Create `src/force_controller/force_controller/controller_manager_client.py` — a reusable class `ControllerManagerClient` that encapsulates all controller_manager interactions as native ROS 2 service clients. The class should:
  - Accept a `Node` instance in its constructor
  - Create service clients for `ListControllers`, `LoadController`, `ConfigureController`, and `SwitchController`
  - Provide a `wait_for_services(timeout_sec)` method that uses the node's existing service client `service_is_ready()` / `wait_for_service()` — no subprocess
  - Provide a `list_controller_states()` method that calls `ListControllers` synchronously and returns a `dict[str, str]` of controller name → state
  - Provide a `ensure_controller_loaded(name)` method that checks state via `list_controller_states()`, then calls `LoadController` + `ConfigureController` if needed
  - Provide a `switch_controllers(activate, deactivate)` method that calls `SwitchController` with BEST_EFFORT strictness first, then falls back to BEST_EFFORT=1 on failure
  - All service calls should use `call_async()` + `rclpy.spin_until_future_complete()` pattern to avoid blocking the executor, OR use synchronous `call()` with a timeout — the choice depends on whether the calling context is inside a callback or not
  - Include proper error handling and logging via the node's logger

  **Rationale**: Extracting this into a shared module eliminates the code duplication between `force_controller_node.py` and `emg_force_grasp_bridge.py`, and provides a clean testable interface.

- [ ] **Task 1.2.** Add `controller_manager_msgs` as a dependency in `src/force_controller/package.xml` — add `<depend>controller_manager_msgs</depend>` to the package dependencies.

  **Rationale**: The native service client approach requires importing `controller_manager_msgs/srv/ListControllers`, etc. This package must be declared as a dependency.

### Phase 2: Refactor Force Controller Node

- [ ] **Task 2.1.** In `force_controller_node.py`, remove the module-level functions `_ros()`, `_wait_for_controller_manager()`, `_controller_states()`, `_ensure_controller_loaded()`, `_switch_controllers()`, and `_competitors()`. Remove the `import subprocess` and `import re` statements (if `re` is no longer needed).

  **Rationale**: These are the subprocess-based functions being replaced. Removing them ensures no accidental fallback to the old pattern.

- [ ] **Task 2.2.** In `ForceControllerNode.__init__()`, instantiate a `ControllerManagerClient(self)` and store it as `self._cm_client`. Call `self._cm_client.wait_for_services(timeout_sec=10)` during initialization (or defer to first use with a lazy-init pattern).

  **Rationale**: Creating the service clients during node initialization ensures the DDS participant discovers the controller_manager services early, before any time-critical switching is needed.

- [ ] **Task 2.3.** Refactor `_activate_controller()` (line 478) to use `self._cm_client.switch_controllers()` instead of the module-level `_switch_controllers()`. The call should be wrapped in a try/except as it currently is. Critically, remove the `time.sleep(self._relaxed_wait_s)` at line 511 — if a settling delay is truly needed, it should be implemented as a non-blocking timer-based state machine transition rather than a blocking sleep.

  **Rationale**: The blocking `time.sleep()` in a subscription callback prevents the executor from processing any other callbacks (force data, joint states, status publishing) during the switch. With native service clients, the call completes faster (no DDS discovery overhead), making the sleep less necessary. If a delay is still needed, it should be async.

- [ ] **Task 2.4.** Refactor `_reset_controller()` (line 535) to use `self._cm_client.switch_controllers()` instead of the module-level `_switch_controllers()`.

  **Rationale**: Same fix for the RELEASING path which shows the identical timeout failure.

- [ ] **Task 2.5.** Refactor `destroy_node()` (line 727) to use `self._cm_client.switch_controllers()` instead of the module-level `_switch_controllers()`. Wrap in try/except as currently done.

  **Rationale**: Cleanup path also uses the subprocess pattern.

- [ ] **Task 2.6.** Move `_competitors()` into the `ControllerManagerClient` class or keep it as a module-level constant helper (it doesn't use subprocess, just list comprehension). Either way, ensure it's still accessible.

  **Rationale**: This function is pure logic with no subprocess dependency; it just needs to remain available.

### Phase 3: Refactor EMG Force Grasp Bridge

- [ ] **Task 3.1.** In `emg_force_grasp_bridge.py`, remove the module-level functions `_ros()`, `_wait_for_controller_manager()`, `_controller_states()`, `_ensure_controller_loaded()`, `_switch_controllers()`, and `_competitors()`. Remove `import subprocess` and `import re`.

  **Rationale**: Same cleanup as the force controller.

- [ ] **Task 3.2.** In `EmgForceGraspBridge.__init__()`, instantiate a `ControllerManagerClient(self)` and store it as `self._cm_client`. Replace the `_wait_for_controller_manager(timeout_s=30)` call at line 398 with `self._cm_client.wait_for_services(timeout_sec=30)`.

  **Rationale**: The EMG bridge also waits for the controller_manager during startup; this should use the native client.

- [ ] **Task 3.3.** Refactor `_start_grasp()` (line 536) to use `self._cm_client.switch_controllers()`. Remove or replace the `time.sleep(self._cfg.relaxed_wait_s)` at line 542.

  **Rationale**: Same blocking-sleep-in-callback issue as the force controller.

- [ ] **Task 3.4.** Refactor `_release_hand()` (line 552) to use `self._cm_client.switch_controllers()`.

- [ ] **Task 3.5.** Refactor `destroy_node()` (line 705) to use `self._cm_client.switch_controllers()`.

### Phase 4: Handle the Blocking Sleep Problem

- [ ] **Task 4.1.** Replace the `time.sleep(self._relaxed_wait_s)` in `_activate_controller()` with a non-blocking state machine approach. Introduce a new internal state (e.g., `_hand_phase = "SETTLING"`) and a one-shot timer that fires after `relaxed_wait_s` seconds to proceed with the controller switch. The `_control_tick()` method should skip command publishing during SETTLING.

  **Rationale**: The blocking sleep prevents the executor from processing force data callbacks, joint state callbacks, and status publishing during the critical transition period. A timer-based approach keeps the executor responsive.

- [ ] **Task 4.2.** Apply the same non-blocking pattern to `emg_force_grasp_bridge.py`'s `_start_grasp()` method, replacing `time.sleep(self._cfg.relaxed_wait_s)` at line 542.

### Phase 5: Testing

- [ ] **Task 5.1.** Create a unit test for `ControllerManagerClient` that verifies the service call construction (request fields, strictness values, fallback logic) using mock service clients. The test should NOT require a running ROS 2 system.

  **Rationale**: The controller switching logic has specific behavior (BEST_EFFORT=2 first, fallback to BEST_EFFORT=1) that must be preserved. A unit test ensures the refactored code maintains this behavior.

- [ ] **Task 5.2.** Verify `colcon build --packages-select force_controller` succeeds after the refactor.

- [ ] **Task 5.3.** Run the existing hardware test script (`tests/test2_hw_force_controller/hw_test.py` if it exists) or manually test the force controller activation/deactivation cycle with the Mia Hand driver.

- [ ] **Task 5.4.** Test the failure scenario: verify that when the controller_manager is temporarily unavailable, the force controller logs a clear error and does not crash, matching the current error-handling behavior.

---

## Verification Criteria

1. **No subprocess calls**: `grep -r "subprocess" src/force_controller/ scripts/emg_force_grasp_bridge.py` returns zero matches
2. **No blocking sleeps in callbacks**: No `time.sleep()` calls remain in `_activate_controller()`, `_reset_controller()`, `_start_grasp()`, or `_release_hand()`
3. **Controller switching works**: The force controller successfully switches between position and velocity controllers during GRASPING entry and RELEASING, verified by log output
4. **Timeout resilience**: When the controller_manager is slow (simulated by adding latency), the native service client completes within the existing timeout bounds without the 3-second-per-subprocess overhead
5. **Build succeeds**: `colcon build --packages-select force_controller` completes without errors
6. **Backward compatibility**: The controller switching behavior (strictness fallback, load+configure before activate) is preserved

---

## Potential Risks and Mitigations

1. **`controller_manager_msgs` not installed in the Docker image**
   - **Risk**: The package may not be present in the current Docker image since the subprocess approach never needed it as a Python import.
   - **Mitigation**: Verify the package is available via `ros2 pkg list | grep controller_manager_msgs`. If missing, add `ros-humble-controller-manager-msgs` to the Dockerfile. It's almost certainly already installed as a transitive dependency of `ros2controlcli`.

2. **Synchronous service calls block the executor**
   - **Risk**: Using `client.call(request)` synchronously within a callback still blocks the single-threaded executor, just like the subprocess approach. The blocking is shorter (no DDS discovery) but still present.
   - **Mitigation**: Use `rclpy.spin_until_future_complete()` from within the callback, or better yet, restructure the controller switch as an async state machine using `call_async()` + done callbacks. For the initial fix, synchronous calls are acceptable since they eliminate the DDS discovery overhead (the dominant cost). The async refactor can be a follow-up.

3. **Service client not ready at startup**
   - **Risk**: If the controller_manager node hasn't started yet when the force controller initializes, the service clients won't connect.
   - **Mitigation**: Use lazy initialization — create the clients at startup but call `wait_for_service()` before first use. The current code already has `_wait_for_controller_manager()` for this purpose; the native version just uses `client.wait_for_service()` instead.

4. **Breaking the EMG bridge**
   - **Risk**: The EMG bridge runs in a different context (launched via `ExecuteProcess` in a launch file) and may have different timing requirements.
   - **Mitigation**: The `ControllerManagerClient` class is designed to be reusable with any `Node` instance. The EMG bridge already has the same startup wait pattern (`_wait_for_controller_manager(timeout_s=30)` at line 398), so the migration is straightforward.

5. **The `relaxed_wait_s` sleep serves a hardware purpose**
   - **Risk**: The 0.5s sleep at line 511 may be needed for the Mia Hand hardware to settle after receiving zero-velocity + open-position commands before switching controllers. Removing it could cause hardware issues.
   - **Mitigation**: Keep the delay but implement it non-blockingly via a one-shot timer or state machine. The delay duration stays the same; only the blocking nature changes.

---

## Alternative Approaches

1. **Quick mitigation only (increase timeouts + retry)**: Increase `_wait_for_controller_manager` timeout from 5s to 15s, increase per-attempt timeout from 3s to 10s, add retry logic in `_activate_controller`. This is a 10-line change that reduces failure frequency but doesn't fix the root cause.
   - **Trade-off**: Minimal risk, but the fundamental fragility remains. Each controller switch still takes 5-30 seconds of subprocess spawning.

2. **Use `ros2cli` Python API directly**: Instead of spawning `ros2` as a subprocess, import `ros2service.api` and call services through the Python API. This avoids process spawning but still creates separate DDS participants.
   - **Trade-off**: Less fragile than subprocess but still slower than native rclpy clients. More complex than the recommended approach.

3. **Move controller switching to a separate thread**: Keep subprocess calls but run them in a `threading.Thread` to avoid blocking the executor.
   - **Trade-off**: Adds threading complexity, doesn't solve the DDS discovery overhead, and introduces race conditions with the executor. Not recommended.

4. **Use `ros2_control` C++ controller manager API**: Rewrite the force controller as a C++ ros2_control controller that can switch controllers natively.
   - **Trade-off**: Major rewrite, changes the entire architecture. The Python node approach is simpler and sufficient.

---

## Recommended Approach

**Phase 1-3 (native service clients)** is the primary fix. Phase 4 (non-blocking sleep) is recommended but can be deferred to a follow-up if the blocking sleep proves harmless after the subprocess overhead is eliminated. Phase 5 (testing) is essential.

The estimated complexity is **medium** — the logic is straightforward (replace subprocess calls with service client calls), but care must be taken with the synchronous vs. async calling pattern and the blocking sleep replacement.
