# Autopilot Bugfile

Repository-specific debugging memory for Autopilot-managed work.

Each entry records error-message fixes that should help future agents avoid rediscovering the same root cause.

## Entry format

```markdown
### <symptom or error text>

- Date: YYYY-MM-DD
- Batch: <autopilot batch number>
- Beads: <bead ids>
- Components/files: `<path>`, `<path>`
- Root cause: <why it failed>
- Fix: <what changed>
- Verification: `<command>` → <observed result>
- Related: <similar entries or none>
```

## Entries

### Isolated worker infrastructure returns 500 Internal Server Error

- Date: 2026-06-18
- Beads: mvp-1dc.2, mvp-1dc.3, mvp-1dc.4, mvp-1dc.5, mvp-1dc.6, mvp-1dc.7
- Components/files: `tests/mia_haptic_force_test/`, `scripts/mia_haptic_force_test/`, `Makefile`
- Root cause: The `autopilot-worker`, `task`, and other OMP task subagents with `isolated: true` consistently return `500 Internal server error` from the agent platform. The same agents also fail with `isolated: false`. The root cause is a platform-side issue, not a code issue.
- Fix: Fall back to direct implementation by the parent agent. When work needs to be isolated, use git worktrees (`git worktree add /tmp/wt-<name> -b autopilot/<name>`) and have the parent commit and merge the results.
- Verification: Direct implementation produced 97 passing tests, 4 skipped (rclpy-dependent). All 7 child beads closed. Worktrees merged cleanly.
- Related: None.

### PTY suite used undefined STAGE_* constants and bare stage/hold_mode strings

- Date: 2026-06-18
- Beads: mvp-1dc.2
- Components/files: `tests/mia_haptic_force_test/offline_suite.py`
- Root cause: The harness referenced `STAGE_COMPLETE`, `STAGE_FAULT`, `STAGE_FORCE_HOLD`, `STAGE_WAITING_FOR_ACTIVATION` which were never defined in `constants.py`. The actual values live on the `Stage` enum (`Stage.COMPLETE.value`, etc.). The code also used bare strings like `"rotating_to_vertical"`, `"force"`, `"wrist"` which should use the enum values for type safety.
- Fix: Replaced all bare strings with `Stage.*.value` and `HoldControl.*.value`. Added `HoldControl` to the import block.
- Verification: `python3 -c "import ast; ast.parse(open('tests/mia_haptic_force_test/offline_suite.py').read())"` → "offline_suite syntax OK". `bash -n scripts/test_mia_haptic_force_offline.sh` → no errors.
- Related: None.

### hand_simulation.py patch duplicated _resolve_finger_command and _step_finger

- Date: 2026-06-18
- Beads: mvp-1dc.1
- Components/files: `scripts/mia_haptic_force_test/common/hand_simulation.py`
- Root cause: An initial edit replaced the `_step_finger` function signature but left the body, creating a broken file where the function had no signature and the body was duplicated. A second edit added duplicate code in `step_hand_simulation`.
- Fix: Rewrote the entire file from scratch with the correct structure: `_step_finger` with keyword-only args, `step_hand_simulation` that calls it once per finger, and a `default_sim_config()` helper for fallback values.
- Verification: `python3 -m pytest tests/mia_haptic_force_test/test_hand_simulation.py -q` → 19 passed.
- Related: None.

### logger_node.py edit removed the class definition line

- Date: 2026-06-18
- Beads: mvp-1dc.5
- Components/files: `scripts/mia_haptic_force_test/logger_node.py`
- Root cause: Multiple incremental edits to add the `prune_run_dirs` import and call accidentally removed the `class LoggerNode(Node):` line and parts of the docstring, leaving the file syntactically valid but semantically broken.
- Fix: Restored the class definition and docstring lines in a single edit, then added the retention call after `self._run_dir.mkdir(...)`.
- Verification: `python3 -c "import ast; ast.parse(open('scripts/mia_haptic_force_test/logger_node.py').read())"` → "logger_node syntax OK". `python3 -m pytest scripts/mia_haptic_force_test/test_conversions.py -q` → 3 passed.
- Related: None.

### CsvLogger edit duplicated run_id/run_dir and broke __init__ structure

- Date: 2026-06-18
- Beads: mvp-1dc.5
- Components/files: `scripts/mia_haptic_force_test.py`
- Root cause: An edit to add `prune_run_dirs` to the monolithic `CsvLogger.__init__` inserted the new code before the `def __init__` line, creating duplicate `self.run_id`/`self.run_dir` assignments and a broken `__init__` body.
- Fix: Replaced lines 453-470 with a single clean `__init__` method: `def __init__(self, cfg: Config) -> None:`, the prefix/stamp/run_id/run_dir/mkdir lines, the retention call, then the snapshot and CSV setup.
- Verification: `python3 -c "import ast; ast.parse(open('scripts/mia_haptic_force_test.py').read())"` → "monolith syntax OK".
- Related: None.

### terminal_ui_node.py _write method and _build_snapshot method were missing

- Date: 2026-06-18
- Beads: mvp-1dc.3
- Components/files: `scripts/mia_haptic_force_test/terminal_ui_node.py`
- Root cause: The TUI rework required adding `_build_snapshot()` and refactoring `_write()` to accept an injected sink. The initial edit also needed `self._run_id` and `self._gesture_label` to be initialized in `__init__`, and a duplicate `# ── UI helpers ──` header was created.
- Fix: Added the missing init lines (`self._run_id`, `self._gesture_label`), added `_build_snapshot()` and refactored `_write()` with an optional `sink` parameter, removed the duplicate header.
- Verification: `python3 -c "import ast; ast.parse(open('scripts/mia_haptic_force_test/terminal_ui_node.py').read())"` → "terminal_ui_node syntax OK". `python3 -m pytest tests/mia_haptic_force_test/test_tui_smoke.py -q` → 16 passed.
- Related: None.

### keyboard_emg_node.py had broken string literal and duplicate class definition

- Date: 2026-06-18
- Beads: mvp-1dc.3
- Components/files: `scripts/mia_haptic_force_test/keyboard_emg_node.py`
- Root cause: The initial write of the file had `if __name__ == "__main__ and __package__ is None and ...` with a missing closing quote, causing a `SyntaxError`. The subsequent edit to import from `common.keyboard_idle` left a duplicate `class KeyboardEmgNode(Node):` line.
- Fix: Fixed the string literal to `if __name__ == "__main__" and ...` and removed the duplicate class definition.
- Verification: `python3 -c "import ast; ast.parse(open('scripts/mia_haptic_force_test/keyboard_emg_node.py').read())"` → "keyboard_emg_node syntax OK".
- Related: None.

### keyboard_gesture_state import failed because keyboard_emg_node imports rclpy at module level

- Date: 2026-06-18
- Beads: mvp-1dc.7
- Components/files: `scripts/mia_haptic_force_test/keyboard_emg_node.py`, `scripts/mia_haptic_force_test/common/keyboard_idle.py`
- Root cause: The topic contract tests tried to import `keyboard_gesture_state` from `keyboard_emg_node`, but that module imports `rclpy` at module level, which fails in the local test environment (rclpy is only available in the Docker container). This made the pure helper untestable without ROS.
- Fix: Moved `keyboard_gesture_state` and `DEFAULT_KEY_IDLE_TIMEOUT_S` to `scripts/mia_haptic_force_test/common/keyboard_idle.py` (no rclpy import). Updated `keyboard_emg_node.py` to import from the common module. Updated the test to import from the common module.
- Verification: `python3 -m pytest tests/mia_haptic_force_test/test_topic_contracts.py -q` → 21 passed, 1 skipped.
- Related: None.
