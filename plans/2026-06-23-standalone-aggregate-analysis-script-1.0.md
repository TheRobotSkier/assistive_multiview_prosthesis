# Standalone Aggregate Log Analysis Script

## Objective

Create a standalone `scripts/aggregate_analysis.py` that scans all host logs and Jetson logs in `logs/`, pairs them by timestamp, and produces a single summary report with easy-to-understand percentage metrics for pipeline health across the entire log corpus.

## Architecture

The script is a pure-stdlib Python script (no ROS dependencies) with three layers:

1. **Parsing layer** — Reuses the regex patterns and dataclass structures already defined in `scripts/analyze_log.py` via direct import. For Jetson logs, adds a lightweight parser that extracts `[DIAG]`, `[MARKER_REJECT]`, and `[DRIFT-INCIDENT]` lines using the same regexes already defined in `analyze_log.py` (lines 115-128: `RE_VIO_DIAG`, `RE_MARKER_REJECT`, `RE_DRIFT_INCIDENT`).

2. **Matching layer** — Pairs host logs to Jetson logs by extracting timestamps from filenames and finding the closest match within a configurable window.

3. **Reporting layer** — Pools metrics across all matched pairs and produces both a human-readable text report and an optional JSON output.

## Implementation Plan

### Phase 1: Script Skeleton and Imports

- [ ] **1.1.** Create `scripts/aggregate_analysis.py` with standard library imports (`argparse`, `glob`, `os`, `re`, `sys`, `json`, `datetime`, `collections.defaultdict`, `dataclasses`).

- [ ] **1.2.** Import parsing infrastructure from the existing analyzer:
  - Import `parse_log`, `LogReport`, `DiagBlock` from `analyze_log`
  - Import regex constants: `RE_VIO_DIAG`, `RE_MARKER_REJECT`, `RE_DRIFT_INCIDENT`, `RE_LOG_LINE`, `RE_DIAG_CHAIN_LINE`
  - Import helper: `_find_sibling_jsonl` (to reuse its timestamp-matching logic pattern)
  - Use `sys.path.insert(0, os.path.dirname(__file__))` to ensure the import works regardless of CWD

### Phase 2: Jetson Log Parser

- [ ] **2.1.** Define `JetsonReport` dataclass with fields:
  ```
  path: str
  head_markers: int = 0          # last cumulative value from [DIAG] head
  head_corrections: int = 0      # last cumulative value from [DIAG] head
  arm_markers: int = 0           # last cumulative value from [DIAG] arm
  arm_corrections: int = 0       # last cumulative value from [DIAG] arm
  head_vio_status: str = "unknown"
  arm_vio_status: str = "unknown"
  head_pos_norm: float = 0.0
  arm_pos_norm: float = 0.0
  marker_rejections: list = field(default_factory=list)   # list of MarkerRejection
  drift_incidents: list = field(default_factory=list)     # list of DriftIncident
  t_start: float | None = None
  t_end: float | None = None
  ```

- [ ] **2.2.** Implement `parse_jetson_log(path: str) -> JetsonReport`:
  - Read file line by line
  - Match against `RE_LOG_LINE` to extract level, timestamp, logger, message
  - For `[DIAG] markers_detected=N corrections=M/N` lines (matched by `RE_VIO_DIAG`):
    - Determine head vs arm from logger name (contains "arm_phase2" or "arm" → arm, else head)
    - Store cumulative values — **use the maximum seen**, not the last value, to handle node restarts where counters reset to 0. If a reset is detected (current < previous), add the previous max to a running total and start tracking from the new baseline. Rationale: the ArUco node crashes and restarts mid-run (documented issue), which resets cumulative counters.
  - For `[MARKER_REJECT] side=X type=Y val=Z lim=W` lines (matched by `RE_MARKER_REJECT`):
    - Append to `marker_rejections` list
  - For `[DRIFT-INCIDENT] side=X reason=Y pos_norm=Z speed=W` lines (matched by `RE_DRIFT_INCIDENT`):
    - Append to `drift_incidents` list
  - Track `t_start` / `t_end` from timestamps

### Phase 3: Host-Jetson Log Matching

- [ ] **3.1.** Implement `extract_timestamp(filename: str) -> str | None`:
  - Extract `YYYYMMDD_HHMMSS` from filenames like:
    - `host-log-20260620_180719.txt` → `"20260620_180719"`
    - `jetson-run-jetson-debug-20260620_180850.txt` → `"20260620_180850"`
    - `host-log-v29.txt` → `None` (no timestamp — old naming convention)
    - `jetson-log-v16.txt` → `None`

- [ ] **3.2.** Implement `find_jetson_pair(host_path: str, jetson_paths: list[str]) -> str | None`:
  - Extract timestamp from host filename
  - If host has no timestamp (old `v*` naming): return `None` (cannot match reliably)
  - Find the Jetson log whose timestamp is closest, within a ±120 second window
  - Jetson container typically starts 0-90s before the host pipeline
  - Return the best match or `None`

- [ ] **3.3.** Build the full paired list in `main()`:
  - Glob all `host-log-*.txt` (excluding `*_analysis.txt`)
  - Glob all `jetson-run-jetson-debug-*.txt` and `jetson-log-*.txt`
  - For each host log, call `find_jetson_pair()`
  - Store as list of `(host_path, jetson_path_or_None)` tuples

### Phase 4: Metric Computation

- [ ] **4.1.** **TF Chain Availability (host-side):**
  For each host `LogReport`, iterate `rep.diag_blocks`. For each block that has chain data:
  - Check if `marker_map -> head_d435i_head_depth_optical_frame` is in `block.chains` with status CONNECTED or OK
  - Check if `marker_map -> arm_d435i_arm_depth_optical_frame` is in `block.chains` with status CONNECTED or OK
  - Increment pooled counters:
    - `head_chain_ok`, `head_chain_total`
    - `arm_chain_ok`, `arm_chain_total`
    - `both_chain_ok`, `both_chain_total` (only count blocks where both chains have data)

- [ ] **4.2.** **Marker Correction Yield (Jetson-side):**
  Pool the cumulative marker/correction counts across all Jetson logs:
  - `total_head_markers += jr.head_markers`
  - `total_head_corrections += jr.head_corrections`
  - Same for arm
  - Compute yield: `corrections / markers * 100` per side and combined

- [ ] **4.3.** **Marker Rejection Breakdown (Jetson-side):**
  From all pooled `MarkerRejection` entries:
  - Total rejections by side (head/arm/marker)
  - Total rejections by type (chi2/translation/rotation/velocity_fit)
  - Mean val vs gate limit per type
  - Compute overall rejection rate: `total_rejections / (total_rejections + total_corrections_accepted) * 100`
  - Separate `side=marker` (MSCKF chi2 gate rejections from C++ node) from `side=head`/`side=arm` (ArUco pose plausibility rejections from Python node) — these are different rejection mechanisms and should not be conflated

- [ ] **4.4.** **Hard Reanchor Frequency (Jetson-side):**
  From all pooled `DriftIncident` entries:
  - Total count, breakdown by `side` and `reason`
  - Compute total monitoring time: sum of `(t_end - t_start)` across all Jetson logs that have DRIFT data
  - Rate: `total_incidents / (total_monitoring_minutes)` → incidents per minute
  - `% runs affected`: `runs_with_incidents / runs_with_drift_data * 100`

- [ ] **4.5.** **Fusion Dual-View Ratio (host-side):**
  From each host `LogReport`, pool `rep.fusion_stats`:
  - `total_dual = sum(s.dual for s in rep.fusion_stats)`
  - `total_cam1_only = sum(s.cam1_only for s in rep.fusion_stats)`
  - Overall ratio: `total_dual / max(total_dual + total_cam1_only, 1) * 100`

### Phase 5: Report Output

- [ ] **5.1.** Implement `print_aggregate_report(metrics: dict, out)` that produces:

```
============================================================================
AGGREGATED PIPELINE HEALTH REPORT
============================================================================
Host logs scanned:     NN
Jetson logs matched:   NN / NN host logs
Monitoring period:     YYYY-MM-DD to YYYY-MM-DD

────────────────────────────────────────────────────────────────────────────
TF CHAIN AVAILABILITY (host-side)
  Pooled from NN host logs, NNNN total 5-second intervals

  Head chain (marker_map → head_depth_optical):  XX.X%  (NNNN / NNNN)
  Arm chain  (marker_map → arm_depth_optical):   XX.X%  (NNNN / NNNN)
  ────────────────────────────────────────────────────────────────────────
  Dual-camera TF uptime (both connected):        XX.X%  (NNNN / NNNN)

────────────────────────────────────────────────────────────────────────────
MARKER CORRECTION YIELD (jetson-side)
  Pooled from NN jetson logs

  Head:  XX.X%  (NNNN corrections / NNNNN markers detected)
  Arm:   XX.X%  (NNNN corrections / NNNNN markers detected)
  ────────────────────────────────────────────────────────────────────────
  Overall marker yield:  XX.X%

────────────────────────────────────────────────────────────────────────────
MARKER REJECTION BREAKDOWN (jetson-side)
  Pooled from NN jetson logs with rejection data

  ArUco pose plausibility gate (side=head/arm):
    Overall rejection rate: XX.X%  (NNNN rejected / NNNN total attempts)
    By type:
      chi2:          NNNN rejections (mean=X vs lim=Y)
      translation:   NNNN rejections (mean=X vs lim=Y)
      rotation:      NNNN rejections (mean=X vs lim=Y)

  MSCKF feature gate (side=marker, C++ node):
    chi2:            NNNN rejections (mean=X vs lim=Y)

────────────────────────────────────────────────────────────────────────────
HARD REANCHORING (jetson-side)
  Pooled from NN jetson logs, XX.X minutes of monitoring

  Total incidents: NNNN
  Rate:            X.X incidents/minute
  % runs affected: XX.X%

  By reason:
    marker_innovation_jump:              NNNN incidents
    hard_reanchor_rotation_implausible:  NNNN incidents

────────────────────────────────────────────────────────────────────────────
FUSION DUAL-VIEW RATIO (host-side)
  Pooled from NN host logs with fusion stats

  Overall dual-view ratio: XX.X%  (NNNN dual / NNNN total published)

────────────────────────────────────────────────────────────────────────────
COVERAGE
  Host logs scanned:           NN
  Host logs with DIAG-CHAIN:   NN
  Host logs with fusion stats: NN
  Jetson logs matched:         NN / NN
  Jetson logs with DIAG:       NN
  Jetson logs with REJECT:     NN
  Jetson logs with DRIFT:      NN
============================================================================
```

- [ ] **5.2.** Implement `write_json_metrics(metrics: dict, path: str)` that writes the same data as structured JSON for CI/regression tracking. Structure:

```json
{
  "generated_at": "2026-06-23T...",
  "coverage": {
    "host_logs_scanned": 68,
    "host_logs_with_chains": 68,
    "host_logs_with_fusion": 73,
    "jetson_logs_matched": 24,
    "jetson_logs_with_diag": 24,
    "jetson_logs_with_reject": 9,
    "jetson_logs_with_drift": 7
  },
  "tf_chain_availability": {
    "head_pct": 45.2,
    "arm_pct": 12.8,
    "dual_pct": 8.1,
    "head_ok_intervals": 340,
    "head_total_intervals": 752,
    "arm_ok_intervals": 96,
    "arm_total_intervals": 752,
    "both_ok_intervals": 61,
    "both_total_intervals": 752
  },
  "marker_yield": {
    "head_pct": 3.1,
    "arm_pct": 0.1,
    "overall_pct": 1.2,
    "head_corrections": 15,
    "head_markers": 483,
    "arm_corrections": 1,
    "arm_markers": 817
  },
  "marker_rejections": {
    "aruco_rejection_rate_pct": 98.9,
    "by_type": { "chi2": {...}, "translation": {...}, "rotation": {...} },
    "msckf_rejections": { "chi2": {...} }
  },
  "hard_reanchoring": {
    "total_incidents": 36,
    "rate_per_minute": 0.8,
    "pct_runs_affected": 100.0,
    "by_reason": { "marker_innovation_jump": 20, "hard_reanchor_rotation_implausible": 16 }
  },
  "fusion_dual_view": {
    "overall_pct": 15.3,
    "total_dual": 412,
    "total_cam1_only": 2278
  }
}
```

### Phase 6: CLI and Entry Point

- [ ] **6.1.** Implement `main()` with `argparse`:
  - Default behavior (no args): scan all logs, print text report to stdout
  - `--json PATH`: also write JSON metrics to the given path
  - `--output PATH`: write text report to file (default: stdout only)
  - `--logs-dir PATH`: override logs directory (default: `../logs` relative to script)
  - `--verbose`: print per-log parsing progress to stderr (useful for debugging)

- [ ] **6.2.** Wire the full pipeline in `main()`:
  1. Glob host and Jetson logs
  2. Parse all host logs with `parse_log()` (reuse from `analyze_log`)
  3. Match and parse Jetson logs
  4. Compute all metrics (Phase 4)
  5. Print report (Phase 5)
  6. Optionally write JSON

- [ ] **6.3.** Add progress output to stderr during parsing (since there are 68+ logs):
  ```
  [aggregate] Parsing host log   3/68: host-log-20260618_082830.txt ...
  [aggregate] Parsing jetson log 3/24: jetson-run-jetson-debug-20260618_083015.txt ...
  [aggregate] Computing metrics...
  [aggregate] Done.
  ```

## Verification Criteria

- [ ] Script runs with `python3 scripts/aggregate_analysis.py` with no arguments and produces a complete report
- [ ] Coverage numbers match manual counts: 68 host logs, ~24 Jetson logs matchable
- [ ] TF chain availability percentages are within expected range (head should be higher than arm based on known issues)
- [ ] Marker yield is very low (<5%) consistent with the documented ArUco crash and solvePnP flip issues
- [ ] Hard reanchor incidents are non-zero with reasons matching known patterns
- [ ] Fusion dual-view ratio is low (<30%) consistent with TF chain disconnections
- [ ] `--json` output is valid JSON parseable by `json.load()`
- [ ] Script completes in under 30 seconds for the full log corpus
- [ ] Script handles gracefully: host logs with no Jetson pair, Jetson logs with no DIAG/REJECT/DRIFT data, empty logs, malformed lines

## Potential Risks and Mitigations

1. **Import path issues when importing from `analyze_log`**
   Risk: `analyze_log.py` uses `from __future__ import annotations` and has module-level constants that might not import cleanly.
   Mitigation: Use `sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))` before import. If import fails, fall back to duplicating the 3 regex patterns (they are stable constants). Test the import early.

2. **Cumulative counter resets from ArUco node crashes**
   Risk: The documented `UnboundLocalError` crash at `aruco_marker_pose_node.py:997` kills the node on first marker detection. When it restarts (via ROS launch supervision), the `[DIAG]` counters reset to 0. Using the last-seen value would massively undercount.
   Mitigation: Track the maximum cumulative value seen across all `[DIAG]` lines per side per run. If a reset is detected (current markers < previous markers), sum the pre-reset max with the post-reset final value. Log resets in verbose mode.

3. **Jetson log format variations**
   Risk: Older `jetson-log-v*.txt` (7 files) may have different formatting or lack the structured `[DIAG]`/`[MARKER_REJECT]`/`[DRIFT-INCIDENT]` lines entirely.
   Mitigation: The parser silently skips lines that don't match any regex. The coverage section reports how many logs contributed data to each metric, so the user knows the sample size.

4. **`side=marker` MARKER_REJECT lines inflating rejection counts**
   Risk: The C++ `run_subscribe_msckf_marker` node emits `side=marker` for MSCKF feature-track chi2 rejections, which are a completely different mechanism from the ArUco pose plausibility checks (`side=head`/`side=arm`).
   Mitigation: Separate `side=marker` rejections into their own "MSCKF feature gate" subsection in both the text report and JSON. Do not include them in the ArUco rejection rate calculation.

5. **Host logs with no `[DIAG-CHAIN]` data (debug_monitor was off)**
   Risk: Some host logs may have been recorded without `debug_monitor:=true`, so they have no diagnostics blocks.
   Mitigation: The coverage section explicitly reports "Host logs with DIAG-CHAIN: NN" so the denominator is transparent. Metrics are only computed from logs that have the relevant data.

## Alternative Approaches

1. **Integrate into `analyze_log.py` as `--aggregate` mode** (previously planned)
   Pros: Single tool, shared code.
   Cons: Makes an already 1499-line file even larger. User explicitly requested a separate script.
   Decision: Rejected per user request.

2. **Copy-paste regexes instead of importing from `analyze_log`**
   Pros: Zero coupling, script is fully standalone.
   Cons: Regex drift if `analyze_log.py` patterns are updated later.
   Decision: Use import as primary approach, with fallback to copy if import fails. The regexes are stable and unlikely to change.

3. **Process pre-generated `_analysis.txt` files instead of raw logs**
   Pros: Much faster (no re-parsing).
   Cons: The per-log analysis reports don't contain per-interval chain data or raw MARKER_REJECT/DRIFT-INCIDENT lists — they only have summaries. Would lose detail needed for accurate pooling.
   Decision: Rejected — raw log parsing is necessary for accurate aggregation.
