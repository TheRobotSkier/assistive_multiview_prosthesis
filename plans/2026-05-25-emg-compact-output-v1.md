# Simplify EMG Classifier Output

## Objective

Replace the verbose ANSI terminal UI in the EMG classifier with compact output that only prints on gesture transitions and a periodic 5-second status summary. This will reduce EMG log output from ~1800 lines per run to ~20-30 lines.

## Implementation Plan

- [ ] Task 1. Remove the `_draw()` function and all ANSI helper functions from `run_classifier.py`
  - Delete `_bold()`, `_green()`, `_yellow()`, `_cyan()`, `_red()`, `_dim()` at lines 51-56
  - Delete `_UP` and `_CLEAR_LINE` constants at lines 59-60
  - Delete `_prop_bar()` and `_conf_bar()` at lines 63-72
  - Delete `_DISPLAY_LINES` and `_first_draw` at lines 77-78
  - Delete the entire `_draw()` function at lines 81-129
  - Rationale: These are only used by the old terminal UI and won't be needed

- [ ] Task 2. Add a compact `_print_transition()` function
  - Takes the same core info (label, confidence, proportional, probs, gesture_names, frame_count, fps, mode)
  - Prints a single line like: `[emg] REST -> OPEN (conf=86%, prop=0.00, mode=NOT_GRASPING)`
  - Also prints per-class probabilities on the same line: `REST:14% POWER:0% OPEN:86% FLEXION:0% EXTENSION:0%`
  - Only called when the gesture label changes from the previous frame

- [ ] Task 3. Add a periodic status summary using a 5-second timer
  - Track `_last_status_time` in the main loop
  - Every 5 seconds of wall-clock time, print: `[emg] status: REST (conf=100%, prop=0.00, 10.0 Hz, frame=494)`
  - This acts as a heartbeat so you know the classifier is alive even when gesture isn't changing
  - Use `time.monotonic()` to track elapsed time (already imported)

- [ ] Task 4. Update the main inference loop (lines 206-248)
  - Remove the `print("\n" * _DISPLAY_LINES)` at line 204
  - Remove the `_first_draw = True` at line 199
  - Add a `_prev_label` variable initialized to `-1` before the loop
  - Add a `_last_status_time = time.monotonic()` before the loop
  - In the loop body, after classification:
    - If `smoothed_label != _prev_label`: call `_print_transition()`, update `_prev_label`
    - If `time.monotonic() - _last_status_time >= 5.0`: print status line, update `_last_status_time`
  - Remove the call to `_draw()` at line 248

- [ ] Task 5. Clean up startup messages
  - Keep the startup banner, model loading, connection, and buffer-filling messages
  - Simplify ANSI usage in startup messages — keep minimal color (just `_green`/`_red`/`_cyan`/`_yellow`/`_bold` for startup errors/highlights only) or remove color entirely for consistency
  - Since the ANSI helpers will be removed, replace startup colors with plain text or keep only the ones used in startup (they're a small set)

- [ ] Task 6. Update the `KeyboardInterrupt` handler (line 250-251)
  - Remove the extra newlines (`\n\n`) since there's no display area to clear
  - Keep the "Stopped." message as plain text

## Verification Criteria

- [ ] Running the classifier interactively shows only gesture transitions and 5s status summaries
- [ ] Running under ROS 2 launch produces clean, grep-able log output
- [ ] All meaningful information is preserved: gesture name, confidence, proportional, per-class probabilities
- [ ] No ANSI escape codes appear in log files
- [ ] Startup, connect, disconnect messages still appear

## Potential Risks and Mitigations

1. **Losing real-time visual feedback during interactive debugging**
   Mitigation: The 5s status summary + transition logs still give a clear picture of what's happening. If needed for deep debugging, the classifier output at 10 Hz is still published on `/emg/*` ROS topics and can be monitored via `ros2 topic echo`.

2. **Missing brief gesture flickers that don't trigger a "change"**
   Mitigation: The smoother already handles this — smoothed_label only changes when the smoother accumulates enough frames. The transition log captures exactly what the pipeline sees.

## Files Modified

| File | Change |
|------|--------|
| `src/emg_bridge/emg_bridge/scripts/run_classifier.py` | Remove ANSI terminal UI, add compact transition + 5s status output |
