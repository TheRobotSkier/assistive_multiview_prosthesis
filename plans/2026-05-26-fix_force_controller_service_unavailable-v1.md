# Fix Force Controller Failing to Switch Controllers

## Objective

Fix the `Failed to switch to velocity controller: Service /controller_manager/list_controllers is not available` error that occurs when the force controller tries to switch from position to velocity control during the GRASPING state transition. The root cause is that the `ControllerManagerClient` never pre-discovers the controller_manager services, so when the time-critical switch happens (0.5s after GRASPING entry), the services are not yet available.

## Root Cause Analysis

The error flow is:

1. Pipeline transitions APPROACHING → GRASPING (proximity near zone)
2. `_activate_controller()` fires at `force_controller_node.py:370`, starts force streaming, creates a 0.5s one-shot timer
3. `_finish_activate()` fires at `force_controller_node.py:411`, calls `self._cm_client.switch_controllers()`
4. `switch_controllers()` calls `list_controller_states()` → `_call_service()` → `service_is_ready()` returns `False`
5. 5-second `wait_for_service()` fallback also fails → `RuntimeError` raised
6. Error logged, `_controller_active = False`, hand never enters CLOSING phase

**Why services aren't ready**: The `ControllerManagerClient` is created at `force_controller_node.py:249` but `wait_for_services()` is **never called** during initialization. The only place `services_ready()` is checked is in `destroy_node()` (line 690). Without pre-discovery, DDS must discover the controller_manager services on-demand during the time-critical switch — and under load, 5 seconds may not be enough.

The same issue occurs in reverse when transitioning GRASPING → RELEASING → IDLE (`_reset_controller()` at line 459).

## Implementation Plan

- [x] **Task 1. Add `wait_for_services()` call during initialization in `force_controller_node.py`**
  - After `self._cm_client = ControllerManagerClient(self)` at line 249, add a call to `self._cm_client.wait_for_services(timeout_sec=10.0)`.
  - Log a warning if services don't become available (but don't crash — the node should still start and wait for services to appear later).
  - Rationale: Pre-discovers the controller_manager services during node startup, eliminating DDS discovery latency when the time-critical switch is needed. The 10-second timeout is generous enough for DDS discovery but short enough to not block startup indefinitely.

- [x] **Task 2. Add `services_ready()` pre-check in `_finish_activate()` before switching**
  - In `_finish_activate()` at line 418, before calling `switch_controllers()`, check `self._cm_client.services_ready()`.
  - If not ready, call `self._cm_client.wait_for_services(timeout_sec=5.0)` as a fallback.
  - If still not ready, log an error and set `_controller_active = False` (same as current error path).
  - Rationale: Even with startup pre-discovery, services can become transiently unavailable (e.g., controller_manager restart). This provides a safety net at the critical switching point.

- [x] **Task 3. Add `services_ready()` pre-check in `_reset_controller()` before switching**
  - In `_reset_controller()` at line 459, before calling `switch_controllers()`, add the same `services_ready()` check with a short wait fallback.
  - Rationale: The same error occurs when releasing (`Failed to switch to position controller: Service /controller_manager/list_controllers is not available`). The fix is symmetric.

- [x] **Task 4. Verify the fix doesn't break existing tests**
  - Ensure `test/test_controller_manager_client.py` still passes (the `ControllerManagerClient` API is unchanged).
  - Ensure `test/test_force_controller_node.py` (if it exists) still passes.
  - Rationale: The changes are additive (new checks before existing calls) and shouldn't break any existing test contracts.

## Verification Criteria

- [ ] `force_controller_node` starts without error and logs that controller_manager services are ready (or warns if they aren't)
- [ ] When transitioning APPROACHING → GRASPING, the velocity controller switch succeeds (no "Service is not available" error)
- [ ] When transitioning GRASPING → RELEASING → IDLE, the position controller switch succeeds
- [ ] Existing unit tests continue to pass

## Potential Risks and Mitigations

1. **Startup blocks for 10s if controller_manager isn't running yet**
   Mitigation: Log a warning but continue. The `_finish_activate()` fallback (Task 2) will attempt discovery again when the switch is actually needed.

2. **`wait_for_services()` uses `spin_until_future_complete` or similar blocking call from a timer callback context**
   Mitigation: `wait_for_services()` uses `client.wait_for_service()` which is non-spinning (it polls internally). The initialization call at line 249 happens in `__init__` before the executor starts spinning, so it may not block at all — `wait_for_service()` uses a condition variable that works without spinning. If it doesn't work in `__init__`, move the call to a startup timer that fires once after the executor is running.

3. **Double-wait adds latency in the happy path**
   Mitigation: If `wait_for_services()` succeeds at startup (Task 1), the `services_ready()` checks in Tasks 2-3 will return `True` immediately — no additional latency.

## Alternative Approaches

1. **Increase the per-call fallback timeout in `_call_service()` from 5s to 20s**: Simple but wastes time on every failed call. Better to pre-discover once.
2. **Make `switch_controllers()` itself call `wait_for_services()` internally**: Would fix it everywhere but changes the `ControllerManagerClient` API contract (currently it's the caller's responsibility).
3. **Retry the entire `_finish_activate()` on failure**: More complex, adds state machine complexity for little benefit over pre-discovery.

**Recommended approach**: Tasks 1-3 (pre-discovery at startup + safety-net checks at switch points).
