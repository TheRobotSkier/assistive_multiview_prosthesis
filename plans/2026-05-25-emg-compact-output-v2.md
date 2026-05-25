# Simplify EMG Classifier Output (ROS launch only)

## Objective

When the EMG classifier runs under ROS 2 launch (piped stdout), use compact output that only prints on gesture transitions and a periodic 5-second status summary. When running directly in a terminal, keep the existing fancy TUI unchanged.

## Detection

Use `sys.stdout.isatty()` at startup:
- `True` (interactive terminal) → use existing `_draw()` TUI
- `False` (ROS launch / piped) → use compact `_print_compact()` output

## Implementation Plan

- [ ] Task 1. Add a `_print_compact()` function for non-TTY mode
  - Takes: label, prev_label, confidence, proportional, probs, gesture_names, frame_count, fps, mode
  - On gesture change, prints: `[emg] REST -> OPEN (conf=86%, prop=0.00, mode=NOT_GRASPING) REST:14% POWER:0% OPEN:86% FLEXION:0% EXTENSION:0%`
  - Rationale: Captures every meaningful event (gesture transitions) in a single grep-able line

- [ ] Task 2. Add periodic 5-second status summary for non-TTY mode
  - Track `_last_status_time` using `time.monotonic()` (already imported)
  - Every 5 seconds, print: `[emg] status: REST (conf=100%, prop=0.00, 10.0 Hz, frame=494)`
  - Rationale: Acts as heartbeat so you know the classifier is alive even when gesture isn't changing

- [ ] Task 3. Update the main inference loop to branch on display mode
  - Before the loop: detect `_interactive = sys.stdout.isatty()`
  - If `_interactive`: keep existing `_draw()` call (line 248) unchanged
  - If not `_interactive`:
    - Track `_prev_label` (init to `-1`) and `_last_status_time`
    - If `smoothed_label != _prev_label`: call `_print_compact()`, update `_prev_label`
    - If `time.monotonic() - _last_status_time >= 5.0`: print status line, update `_last_status_time`
  - The `print("\n" * _DISPLAY_LINES)` at line 204 should only execute in interactive mode
  - Rationale: Zero change to the interactive experience, clean output only in ROS launch

- [ ] Task 4. No changes to ANSI helpers or `_draw()`
  - Keep all existing functions (`_draw`, `_bold`, `_green`, `_prop_bar`, etc.) exactly as-is
  - They are only called in interactive mode
  - Rationale: Don't break the existing TUI that works well in terminals

## Verification Criteria

- [ ] Running `ros2 run emg_bridge run_classifier` in a terminal shows the existing fancy TUI
- [ ] Running under `make run` (ROS 2 launch) shows only transitions + 5s status summaries
- [ ] No ANSI escape codes appear in log files when run through ROS launch
- [ ] All meaningful info preserved in both modes: gesture, confidence, proportional, per-class probs

## Potential Risks and Mitigations

1. **`isatty()` detection edge case (e.g. `script` command)**
   Mitigation: In practice, ROS 2 launch always pipes stdout, so `isatty()` is reliably False. If someone needs to force one mode, a `--quiet` flag could be added later.

## Files Modified

| File | Change |
|------|--------|
| `src/emg_bridge/emg_bridge/scripts/run_classifier.py` | Add `_print_compact()`, branch main loop on `sys.stdout.isatty()` |
