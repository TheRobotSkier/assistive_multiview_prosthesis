# Iteration 15 — Fix stale log analysis: capture pipeline output + exclude `_analysis.txt` from glob

## What I Saw in the Logs

### Latest replay results (iteration14 run `replay_20260617_043423`)

| Metric | Value | Previous (iter12) | Delta |
|--------|-------|-------------------|-------|
| `/gtsam/head_pose` eff Hz | 11.997 | 12.145 | **−0.148 Hz (−1.2 %)** |
| `/gtsam/arm_pose` eff Hz | 11.997 | 12.145 | **−0.148 Hz (−1.2 %)** |
| `/tf` eff Hz | 30.86 | 30.82 | ~unchanged |
| Host CPU avg | 61.3 % | 60.3 % | +1 % |
| Host CPU max | **92.8 %** | 83.1 % | **+9.7 %** |
| N crashes / errors / TF jumps | 0 | 0 | — |

The iteration14 change (restoring `relinearizeThreshold=0.01`) partially recovered GTSAM rate from 11.565 Hz (iter13) back to 11.997 Hz — but NOT fully to the 12.145 Hz of iteration12. CPU max spiked to 92.8 % (a new high), indicating the tighter threshold increases per-update compute load.

### Stale log analysis — the deeper problem

**The `log_analysis.txt` has been pointing to a stale file (`host-log-20260616_162416_analysis.txt`) for at least 4 iterations** (iter12–iter15). That file is the *analysis output* from a June16 run (111 lines, n/a timespan, 0 nodes started), NOT the raw pipeline log from the current run.

Root cause in `replay_test.py:469`:
```python
"ls -t /prosthesis_ws/logs/host-log-*.txt 2>/dev/null | head -1"
```
This glob matches both raw logs (`host-log-20260616_162416.txt`) and previously generated analysis files (`host-log-20260616_162416_analysis.txt`). Since analysis files have newer mtimes, they always get picked — giving the current test zero diagnostic value.

**Worse**: the replay test discards all pipeline output via `container_exec_bg()` which uses `stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL`. No fresh raw logs are generated at all.

This means:
- The `gtsam_corr` diagnostic (added in iteration12) has NEVER appeared in any replay analysis report
- TF jump events, errors, and rate-decline timeline are invisible
- We have no way to evaluate Goal 1 ("GTSAM factor-graph optimization clearly improves localization precision over raw VIO") from replay results
- Each iteration has been flying blind on the diagnostic front

## Decision for This Iteration

**Problem:** The replay test's log analysis is non-functional — it neither captures pipeline output to a file nor selects the correct file for analysis. This has been silently degrading the diagnostic value of every replay run for 4+ iterations.

Fix both sides:
1. **Capture**: Redirect pipeline output (inside the container) to a timestamped `host-log-<timestamp>.txt` file so analyze_log.py has fresh data to parse
2. **Select**: Fix the glob in `replay_test.py` and `analyze_log.py` to exclude `*_analysis.txt` files

This is the **foundational diagnostic fix** that unblocks all future iterations. Once logs are captured and analyzed properly, every subsequent iteration will see:
- `gtsam_corr(head=0.XXXXm, arm=0.XXXXm)` — the direct measure of GTSAM's correction over VIO
- TF jump events and their per-edge breakdown
- Per-interval effective rates (showing the rate-decline curve)
- Warning/error messages from the current run

### Why this over a performance fix

The GTSAM rate of 12.0 Hz is only 1.2 % below the 12.145 Hz peak. The CPU max of 92.8 % is concerning but not causing crashes or errors. With the stale log issue fixed, future iterations can make targeted performance changes (e.g., smarter relinearization scheduling, ISAM2 parameter tuning) and **measure their actual impact** through the `gtsam_corr` diagnostic and rate timeline.

Without this fix, the auto-iterate loop continues producing replay results with a blind spot for the most important diagnostic — whether GTSAM is actually improving over VIO.

## Changes Made

### 1. `scripts/replay_test.py:233-253` — Capture pipeline output to host-log

**Before:**
```python
launch_cmd = (
    "cd /prosthesis_ws && "
    "ros2 launch prosthesis_launch pipeline.launch.py " + " ".join(LAUNCH_ARGS)
)
```

**After:**
```python
launch_cmd = (
    "cd /prosthesis_ws "
    "&& mkdir -p /prosthesis_ws/logs "
    "&& LOG_FILE=/prosthesis_ws/logs/host-log-$(date +%Y%m%d_%H%M%S).txt "
    "&& echo '=== Logging pipeline output to $LOG_FILE ===' "
    "&& ros2 launch prosthesis_launch pipeline.launch.py "
    + " ".join(LAUNCH_ARGS)
    + " > $LOG_FILE 2>&1"
)
```

The command now creates a timestamped log file path inside the container and redirects all pipeline stdout/stderr there. The log file lands at `/prosthesis_ws/logs/host-log-<datetime>.txt`, which is bind-mounted to `./logs/` on the host. The `container_exec_bg()` still uses `stdout=subprocess.DEVNULL` for the container exec channel, but the shell-level redirection `> $LOG_FILE 2>&1` captures the output before it reaches that channel.

### 2. `scripts/replay_test.py:481-486` — Exclude `_analysis.txt` from log finder

**Before:**
```python
r = container_exec(
    "ls -t /prosthesis_ws/logs/host-log-*.txt 2>/dev/null | head -1",
    ...)
```

**After:**
```python
r = container_exec(
    "ls -t /prosthesis_ws/logs/host-log-*.txt 2>/dev/null "
    "| grep -v '_analysis' "
    "| head -1",
    ...)
```

The `grep -v '_analysis'` filters out previously generated analysis files, ensuring only raw pipeline output logs are considered.

### 3. `scripts/analyze_log.py:652-663` — Exclude `_analysis.txt` from `_latest_log()`

**Before:**
```python
def _latest_log():
    logs = glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt"))
    if not logs:
        return None
    return max(logs, key=os.path.getmtime)
```

**After:**
```python
def _latest_log():
    logs = [f for f in glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt"))
            if not f.endswith("_analysis.txt")]
    if not logs:
        return None
    return max(logs, key=os.path.getmtime)
```

Same fix for standalone `analyze_log.py` usage (without an explicit log path).

### 4. `scripts/analyze_log.py:676-680` — Exclude `_analysis.txt` from `--all` mode

**Before:**
```python
if args.all:
    logs = sorted(glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt")))
```

**After:**
```python
if args.all:
    logs = sorted(
        f for f in glob.glob(os.path.join(LOGS_DIR, "host-log-*.txt"))
        if not f.endswith("_analysis.txt")
    )
```

## Expected Impact

### Diagnostic visibility (unlocks Goal 1 measurement)

Starting with the next replay test run:

1. **`gtsam_corr` values will appear in log_analysis.txt.** The `_log_stats()` method in `gtsam_tracker_node.py:732-745` emits `gtsam_corr(head=0.XXXXm, arm=0.XXXXm)` every 10 s. The `analyze_log.py` known-failure patterns already track this output (though as `gtsam odometry rejected` — the actual correction norms will appear in the raw log's INFO lines). Future iterations can grep for `gtsam_corr` to see if the correction is non-zero (GTSAM actively improving over VIO) or zero (graph broken).

2. **TF jump events will be tracked.** The `pipeline_diagnostics_node` emits `TF JUMP: ...` warnings. The `analyze_log.py` `RE_TF_JUMP` pattern (line 86) matches these and counts them in `tf_jump_events_total`. Previously this always returned 0 because the stale analysis file had no matching content.

3. **Rate timeline available.** The `--plot` mode of `analyze_log.py` generates PNG timelines of per-interval topic rates, showing the rate-decline curve that iteration13 identified as a concern.

4. **Known-failure patterns match correctly.** Patterns like `tsdf GetAllKeyframes timeout`, `gtsam odometry rejected`, and `odom pose jumped` will now detect real occurrences from the current run instead of always returning 0.

### What to check in the next iteration

1. **Verify `host-log-<timestamp>.txt` appears in `logs/`.** After the replay test runs, a new raw log file should exist alongside the existing June16-era logs. Its timestamp should match the replay run time.

2. **Verify `log_analysis.txt` contains real data.** The report should show:
   - Actual time span (not `n/a`)
   - Nodes started with PIDs
   - Severity tally per node
   - Known-failure digest with actual counts
   - DIAGNOSTICS TIMELINE blocks with per-topic rates

3. **Grep the raw log for `gtsam_corr`.** Expected values like `gtsam_corr(head=0.0234m, arm=0.0189m)` — non-zero positive values indicating GTSAM is actively correcting VIO drift.

4. **Check `tf_jump_events_total` in `log_metrics.json`.** Should reflect the actual TF jump count (iter13's manual grep found 5541 jumps on raw VIO edges).

5. **GTSAM output rate** — unchanged by this fix, so expect ~12.0 Hz (matching iter14). The rate is a separate concern for future iterations.

### Relationship to the two goals

- **Goal 1 (localization precision):** This is the foundational diagnostic fix. Without it, we cannot measure whether GTSAM is improving over VIO because `gtsam_corr` and TF jump data are invisible. With it, every future iteration has a feedback loop.
- **Goal 2 (TSDF fusion quality):** Indirect — better log diagnostics mean tighter iteration loops, faster debugging of any fusion artifacts.

### Nothing else changed

- No ISAM2 parameters, factor graph topology, smoother lag, or SE(3) math changed.
- All unit-testable logic paths are untouched.
- The 8 RCLError node-test failures are unchanged (ROS2 test infrastructure).
