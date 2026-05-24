# Fix Segmentation Container Crash & Verify Click Reset

## Objective

Diagnose why the segmentation container (`segmentation-cuda`) crashes intermittently, and verify that clicks are properly reset between multiple segmentation cycles.

---

## Root Cause Analysis

### Problem 1: Segmentation Container Crashes After ~2 Minutes

**Symptom**: The container starts, health check succeeds briefly (`"status":"ok"`, `"model_ready":true`), then becomes unreachable within ~2 minutes. The status shows "starting" indefinitely, and eventually the health endpoint returns "NOT reachable on port 5678".

**Evidence from the user's terminal output**:
1. At T+41s: health check returns `{"status":"ok","model_ready":true,...}` — server is running
2. At T+2min: health check times out / "NOT reachable on port 5678" — server has crashed
3. Volume `segmentation-weights` not found (expected — compose uses bind mount `${HOME}/prosthesis_data/weights:/weights`, not a named volume)

**Root Cause**: The inference server at `src/segmentation/nodes/inference_server.py:107-144` processes `/segment` requests single-threaded (`threaded=False` at line 150). The Flask server is blocking — while one inference request is being processed (which takes 0.6-1.5s per the logs), no other request can be served. But the **crash** (not just slowness) is most likely caused by:

1. **CUDA OOM during inference**: The `_inseg.prediction()` call at `inference_server.py:134` runs a MinkowskiEngine sparse 3D CNN forward pass. If a large cloud arrives (120K+ points), GPU memory may be exhausted. The `/segment` handler has **no try/except** around the inference call (lines 131-143) — an unhandled `torch.cuda.OutOfMemoryError` or `RuntimeError` will crash the Flask worker thread, and since Flask is single-threaded, the entire server becomes unresponsive.

2. **Flask dev server instability**: `app.run()` uses the Werkzeug development server (line 150), which is not production-grade. Under sustained load or after an unhandled exception in a request handler, it can silently stop accepting connections.

3. **No process supervision**: The compose `command` is `["/bin/bash", "-c", "exec python /nodes/inference_server.py"]` (docker-compose.yml:87). If the Python process crashes, the container exits. Docker will restart it (if `restart:` policy is set — currently **it is not**). The container status shows "Up X minutes (starting)" because the health check keeps failing after the crash.

**The key smoking gun**: The user says "it just crashes randomly" — this is consistent with an OOM crash triggered by a large inference request, or a Flask worker crash from an unhandled exception.

### Problem 2: Weights Volume Not Found

**Symptom**: `Volume 'segmentation-weights' not found.` in the status output.

**Root Cause**: This is a **cosmetic issue** in `Makefile:178`. The `segmentation-status` target checks for a named Docker volume called `segmentation-weights`, but the compose file at `docker-compose.yml:91` uses a **bind mount** (`${HOME}/prosthesis_data/weights:/weights`), not a named volume. The named volume `segmentation-weights` was removed in a previous refactor. The `Makefile:163` `clean-volumes` target still references it too. This is harmless but confusing.

### Problem 3: Click Reset Between Multiple Clicks

**Symptom**: The user asks to "verify that we reset between multiple clicks we send."

**Analysis of the reset/click flow**:

1. **twist_propagation_node** publishes reset + clicks in sequence (twist_propagation_node.py:1634-1665):
   - Line 1635: `self._reset_pub.publish(Empty())` — publishes reset
   - Line 1655: `self._click_pub.publish(click)` — publishes original click
   - Lines 1658-1665: publishes synthetic clicks in a loop

2. **segmentation_ros2_node** receives both on the same ROS2 executor thread:
   - `_reset_cb` (line 261-269): clears `_pos_clicks` and `_neg_clicks`, cancels debounce timer
   - `_pos_click_cb` (line 239-248): appends to `_pos_clicks`, schedules debounce timer

3. **The race condition**: Since both are on the same single-threaded executor, they are processed sequentially. The log evidence shows:
   ```
   [twist_propagation] Published segmentation reset (new object)
   [twist_propagation] Published click cluster: 5 clicks ...
   [segmentation_bridge] [+] positive click at ... (cloud frame)
   [segmentation_bridge] Clicks reset.
   ```
   
   The reset arrives **after** the first click! This is because DDS message delivery order is not guaranteed to match publish order when messages are on different topics (`/segmentation/reset` vs `/segmentation/click_positive`). The segmentation node sees the click first, then the reset clears it.

   **Impact**: The first click is added, then immediately cleared by the reset. The subsequent synthetic clicks arrive after the reset and are accumulated correctly. So the first (original) click is **lost** on every segmentation cycle. However, looking more carefully at the logs:
   ```
   [segmentation_bridge] [+] positive click at (0.202, -0.296, 0.159) (cloud frame)
   [segmentation_bridge] Clicks reset.
   [segmentation_bridge] [+] positive click at (0.204, -0.305, 0.147) (cloud frame)
   ```
   
   The first click IS added, then reset clears it, then the remaining clicks are added. With 5 clicks total (1 original + 4 synthetic), after the race, only 4 synthetic clicks survive. The original click (which is at the exact hit point) is lost.

   **This is a real bug** — the original click at the exact hit location is being dropped because DDS delivers the first click before the reset.

---

## Implementation Plan

### Phase 1: Fix segmentation container crash (critical)

- [ ] **1.1** Add a `restart: unless-stopped` policy to the `segmentation-cuda` service in `docker/docker-compose.yml:75-102` (and similarly for `segmentation-cpu`). This ensures the container auto-restarts if the Python process crashes, providing resilience while the root cause is fixed.

- [ ] **1.2** Wrap the inference logic in `/segment` handler (`inference_server.py:131-143`) in a try/except block. Catch `RuntimeError` (covers CUDA OOM), `Exception`, log the error, and return a JSON error response with HTTP 500 instead of letting the exception propagate and crash the Flask worker. The current code at lines 107-144 has no error handling for the actual inference call.

- [ ] **1.3** Add `torch.cuda.empty_cache()` after each inference call (after line 137, before the mask post-processing). This releases GPU memory between requests, preventing OOM accumulation across sequential inference calls.

- [ ] **1.4** Consider replacing Flask's development server with a production-grade WSGI server (e.g., `gunicorn` or `waitress`) with a single worker. The Flask dev server is not designed for production use and can silently fail. Alternatively, add a signal handler and top-level try/except in the `if __name__` block to log crashes.

### Phase 2: Fix the reset/click race condition

- [ ] **2.1** In `twist_propagation_node.py:1634-1665`, add a small sleep (e.g., 10-50ms) between publishing the reset and publishing the first click. This gives DDS time to deliver the reset to subscribers before the clicks arrive. Alternatively, publish the reset on a separate timer callback to guarantee ordering.

- [ ] **2.2** A more robust alternative: in `segmentation_ros2_node._reset_cb`, instead of clearing clicks immediately, set a `_reset_pending` flag. In `_pos_click_cb` and `_neg_click_cb`, if `_reset_pending` is true, clear existing clicks first, then add the new click, then clear the flag. This ensures that even if clicks arrive before the reset message, the reset semantics are preserved. However, this is fragile — better to fix the publish order.

- [ ] **2.3** Best approach: Modify `_reset_cb` to also accept a "generation" counter or timestamp. The twist_propagation node can include a timestamp in the reset message. The segmentation node ignores clicks that arrive before the most recent reset. However, this requires changing the reset message type from `std_msgs/Empty` to a custom message with a timestamp, which is more invasive.

### Phase 3: Fix the weights volume status check (cosmetic)

- [ ] **3.1** Update `Makefile:178` to check the bind mount path (`${HOME}/prosthesis_data/weights`) instead of looking for a named Docker volume `segmentation-weights`. The command should be something like `ls -lh ${HOME}/prosthesis_data/weights/ 2>/dev/null || echo "Weights directory not found at ${HOME}/prosthesis_data/weights"`.

- [ ] **3.2** Remove `segmentation-weights` from the `clean-volumes` target at `Makefile:163` since it no longer exists as a named volume.

---

## Verification Criteria

1. **Segmentation stability**: After starting `segmentation-cuda`, the health endpoint remains reachable for 10+ minutes of sustained inference requests without crashing.
2. **OOM resilience**: Sending a large cloud (100K+ points) to `/segment` returns a valid response (or a clear 500 error) without crashing the server. Subsequent requests still succeed.
3. **Click reset correctness**: In the segmentation node logs, the reset message should appear BEFORE any click messages for a new segmentation cycle. The original hit-point click should be included in the inference.
4. **Weights status accuracy**: `make segmentation-status` correctly reports whether weights exist at the bind mount path.

---

## Potential Risks and Mitigations

1. **Adding `restart: unless-stopped` masks the real crash cause**
   Mitigation: This is intentional — it provides resilience. The try/except in Phase 1.2 addresses the root cause. The restart policy is a safety net for any remaining edge cases.

2. **Adding a sleep between reset and clicks adds latency**
   Mitigation: 10-50ms is negligible compared to the 0.6-1.5s inference time. The debounce timer in the segmentation node already adds 20ms. This is the simplest fix with the least risk.

3. **`torch.cuda.empty_cache()` may slow down subsequent inferences**
   Mitigation: The cache empty only frees memory that's not in use. PyTorch will re-allocate on the next inference, but this is fast (sub-millisecond) compared to the CNN forward pass. The benefit of preventing OOM far outweighs the cost.

4. **Flask dev server may still be unstable under load**
   Mitigation: The try/except wrapping and restart policy should handle most cases. If Flask still proves unreliable, a switch to waitress (pure Python, no dependencies beyond pip install) can be done in a follow-up.

---

## Alternative Approaches

1. **Use waitress instead of Flask dev server**: Add `pip install waitress` to the Dockerfile and change `app.run()` to `waitress.serve(app, host='127.0.0.1', port=PORT)`. This is a production-grade WSGI server that handles errors gracefully. Trade-off: adds a dependency, but it's lightweight and well-maintained.

2. **Add a watchdog process inside the container**: Use a process manager like `supervisord` to restart the Python server if it crashes. Trade-off: adds complexity to the Docker image. The Docker `restart:` policy achieves the same effect at the container level.

3. **Queue-based click handling**: Instead of publishing reset and clicks as separate messages, publish a single "segmentation request" message containing all clicks and a reset flag. This eliminates the DDS ordering issue entirely. Trade-off: requires changes to both the twist_propagation and segmentation nodes, plus a new message type.
