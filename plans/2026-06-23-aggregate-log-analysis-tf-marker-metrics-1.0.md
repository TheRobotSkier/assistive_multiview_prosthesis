# Aggregated Log Analysis: TF Health + Marker Yield + Reanchor Rate

## Objective

Add an `--aggregate` mode to `scripts/analyze_log.py` that processes all host logs and their sibling Jetson logs to produce a single summary report with easy-to-understand percentage metrics for:
1. TF chain availability (host-side)
2. Marker correction yield (Jetson-side)
3. Hard reanchor frequency (Jetson-side)
4. Fusion dual-view ratio (host-side)

## Data Coverage Assessment

### Host logs (68+ files, `host-log-*.txt`)
- `[DIAG-CHAIN]` blocks: TF chain CONNECTED/DISCONNECTED every 5s — **68 files have this**
- `Stats: published=N (dual=X, cam1_only=Y)`: Fusion dual-view ratio — **73 files have this**
- `[RELAY-STATUS]`, odom suppression, QoS warnings — good coverage

### Jetson logs (~30+ files, `jetson-run-jetson-debug-*.txt`)
- `[DIAG] markers_detected=N corrections=M/N`: VIO marker yield — **24 files**
- `[MARKER_REJECT] side=X type=Y val=Z lim=W`: Rejection breakdown — **9 files** (newer runs only)
- `[DRIFT-INCIDENT] side=X reason=Y`: Hard reanchor events — **7 files** (newer runs only)

**Key constraint:** Not all host logs have a matching Jetson log. The aggregate must handle missing Jetson data gracefully and report coverage alongside metrics.

## Implementation Plan

### Phase 1: Jetson Log Parser

- [ ] **1.1.** Add `parse_jetson_log(path)` function to `scripts/analyze_log.py` that extracts VIO/marker metrics from Jetson text logs. This mirrors `parse_log()` but targets Jetson-side structured log lines. The parser must handle the Jetson log format which uses `[node_name-N] [LEVEL][epoch.ts][logger]: message` (slightly different bracket spacing from host logs).

- [ ] **1.2.** Extract `[DIAG] markers_detected=N corrections=M/N` lines from Jetson logs. These are cumulative counters — the **last** line per side (head/arm) gives the final totals. Store as `JetsonReport.head_markers`, `head_corrections`, `arm_markers`, `arm_corrections`. Also extract `vio_status` and `pos_norm` from the same line.

- [ ] **1.3.** Extract `[MARKER_REJECT]` lines from Jetson logs. Aggregate by `side` (head/arm/marker) and `type` (chi2, translation, rotation, velocity_fit). Store total counts and mean values per type per side. These represent the detailed breakdown of WHY corrections failed.

- [ ] **1.4.** Extract `[DRIFT-INCIDENT]` lines from Jetson logs. Count total incidents, break down by `side` and `reason` (e.g. `marker_innovation_jump`, `hard_reanchor_rotation_implausible`). Store as a list of incidents with timestamps for rate computation.

- [ ] **1.5.** Create a `JetsonReport` dataclass to hold: `path`, `head_markers`, `head_corrections`, `arm_markers`, `arm_corrections`, `head_vio_status`, `arm_vio_status`, `marker_rejections` (list), `drift_incidents` (list), `t_start`, `t_end`.

### Phase 2: Host-Jetson Log Matching

- [ ] **2.1.** Add `_find_sibling_jetson_log(host_log_path)` function modeled on the existing `_find_sibling_jsonl()` at `scripts/analyze_log.py:1222-1249`. Extract the timestamp from the host log filename (e.g. `host-log-20260620_180719.txt` → `20260620_180719`) and find the closest `jetson-run-jetson-debug-*.txt` by timestamp. The Jetson log typically starts slightly before the host log (container starts first).

- [ ] **2.2.** Also check for `jetson-log-v*.txt` files (older naming convention) as a fallback when no `jetson-run-*` match is found.

### Phase 3: Aggregated Metrics Computation

- [ ] **3.1.** **TF Chain Availability (host-side):** For each host log, iterate `rep.diag_blocks` and for each block check if `marker_map -> head_d435i_head_depth_optical_frame` and `marker_map -> arm_d435i_arm_depth_optical_frame` are both in `block.chains` with status CONNECTED/OK. Pool across all logs:
  - `head_chain_ok_intervals` / `head_chain_total_intervals` → Head availability %
  - `arm_chain_ok_intervals` / `arm_chain_total_intervals` → Arm availability %
  - `both_connected_intervals` / `total_intervals_with_chain_data` → **Dual-camera TF uptime %**

- [ ] **3.2.** **Marker Correction Yield (Jetson-side):** Pool the last-seen cumulative values across all Jetson logs:
  - `total_head_corrections` / `total_head_markers` → Head yield %
  - `total_arm_corrections` / `total_arm_markers` → Arm yield %
  - Combined: `(head_corr + arm_corr)` / `(head_markers + arm_markers)` → **Overall marker yield %**

- [ ] **3.3.** **Marker Rejection Rate (Jetson-side):** From pooled MARKER_REJECT data, compute:
  - Total rejections / (total rejections + total corrections accepted) → **Rejection rate %**
  - Breakdown by type: chi2 vs translation vs rotation vs velocity_fit
  - Breakdown by side: head vs arm

- [ ] **3.4.** **Hard Reanchor Frequency (Jetson-side):** From pooled DRIFT-INCIDENT data:
  - Total incidents / total monitoring time → **Incidents per minute**
  - Breakdown by reason: `marker_innovation_jump` vs `hard_reanchor_rotation_implausible`
  - Percentage of runs with at least one incident → **% runs affected**

- [ ] **3.5.** **Fusion Dual-View Ratio (host-side):** Already extracted per-log in `to_metrics()`. Pool across all logs:
  - `sum(dual)` / `max(sum(dual) + sum(cam1_only), 1)` → **Overall dual-view ratio %**

### Phase 4: Aggregate Report Output

- [ ] **4.1.** Add `print_aggregate_summary()` function that produces a clean, presentation-ready summary. Format:

```
============================================================================
AGGREGATED PIPELINE HEALTH — N host logs, M jetson logs matched
============================================================================

TF CHAIN AVAILABILITY (host-side, from DIAG-CHAIN blocks)
  Head chain:  XX%  (N_ok / N_total intervals across R runs)
  Arm chain:   YY%  (N_ok / N_total intervals across R runs)
  Both (dual): ZZ%  (N_both / N_total intervals across R runs)

MARKER CORRECTION YIELD (jetson-side, from [DIAG] cumulative counters)
  Head:  XX%  (N_corr / N_markers across R runs with data)
  Arm:   YY%  (N_corr / N_markers across R runs with data)
  Total: ZZ%  (combined)

MARKER REJECTION BREAKDOWN (jetson-side, from [MARKER_REJECT] lines)
  Overall rejection rate: XX%  (N_rejected / N_total_attempts)
  By type:
    chi2:         NNN rejections (mean val=X vs lim=Y)
    translation:  NNN rejections (mean val=X vs lim=Y)
    rotation:     NNN rejections (mean val=X vs lim=Y)

HARD REANCHORING (jetson-side, from [DRIFT-INCIDENT] lines)
  Total incidents: NNN across R runs
  Rate: X.X incidents/minute of monitoring
  % runs affected: XX%
  Top reasons:
    marker_innovation_jump:              NN incidents
    hard_reanchor_rotation_implausible:  NN incidents

FUSION DUAL-VIEW RATIO (host-side, from Stats lines)
  Overall: XX%  (N_dual / N_dual+N_cam1_only across R runs)

COVERAGE
  Host logs scanned:      NN
  Host logs with chains:  NN
  Host logs with fusion:  NN
  Jetson logs matched:    NN
  Jetson logs with DIAG:  NN
  Jetson logs with REJ:   NN
  Jetson logs with DRIFT: NN
```

- [ ] **4.2.** Add `--aggregate` CLI flag to `main()`. When set, it:
  1. Globs all `host-log-*.txt` (excluding `*_analysis.txt`)
  2. For each, calls `parse_log()` and attempts `_find_sibling_jetson_log()`
  3. If a Jetson log is found, calls `parse_jetson_log()`
  4. Pools all metrics and calls `print_aggregate_summary()`
  5. Writes to `logs/aggregate_report.txt` and echoes to stdout

- [ ] **4.3.** Add `--aggregate-json` option to write the pooled metrics as JSON for regression tracking. Structure mirrors `to_metrics()` but with aggregate fields.

### Phase 5: Per-Log Chain Availability (enhancement to existing single-log report)

- [ ] **5.1.** In `print_timeline()` at `scripts/analyze_log.py:762-776`, enhance the existing TF chain connectivity section to compute and display the availability percentage alongside the existing FLIPPED/DISCONNECTED/OK classification. Change from:
  ```
  marker_map -> head_d435i_head_depth_optical_frame: FLIPPED (5 OK, 11 DISCONNECTED of 16)
  ```
  to:
  ```
  marker_map -> head_d435i_head_depth_optical_frame: 31% available (5 OK, 11 DISCONNECTED of 16)
  ```

- [ ] **5.2.** Add a "Dual-camera uptime" line to the single-log report showing the percentage of intervals where both chains were simultaneously connected.

## Verification Criteria

- [ ] `python3 scripts/analyze_log.py --aggregate` produces a report with all 5 metric sections
- [ ] Coverage numbers are accurate (manual spot-check against `ls logs/`)
- [ ] Head/arm marker yield percentages match hand-computed values from at least 2 Jetson logs
- [ ] TF chain availability matches manual count from at least 2 host logs
- [ ] Runs with no Jetson log are handled gracefully (no crash, coverage shows 0 matched)
- [ ] Single-log `--aggregate` is not set still works with the enhanced chain availability display
- [ ] `--aggregate-json` produces valid JSON with all scalar metrics

## Potential Risks and Mitigations

1. **Jetson log format variations:** Older `jetson-log-v*.txt` files may use different formatting than `jetson-run-jetson-debug-*.txt`.
   Mitigation: The regexes (`RE_VIO_DIAG`, `RE_MARKER_REJECT`, `RE_DRIFT_INCIDENT`) already handle the structured `[LEVEL]` prefix variations. Test against both naming conventions.

2. **Timestamp matching failures:** Host and Jetson logs may not align perfectly if runs were started manually at different times.
   Mitigation: Use fuzzy matching (±60s window) and report unmatched logs explicitly in the coverage section. Metrics are computed independently per source — missing Jetson data doesn't break host metrics and vice versa.

3. **Cumulative counter resets across node restarts:** If the aruco node crashes and restarts mid-run (as documented in the 06-20 plan), the `[DIAG]` counters reset. Using the last-seen value would undercount.
   Mitigation: Track the maximum cumulative value seen across all `[DIAG]` lines per side, not just the last one. If a reset is detected (current value < previous), log it and use the sum of pre-reset max + post-reset final.

4. **MARKER_REJECT `side=marker` ambiguity:** The C++ `run_subscribe_msckf_marker` node emits `side=marker` (not head/arm). These are MSCKF feature rejections, not ArUco marker pose rejections.
   Mitigation: Separate `side=marker` (MSCKF chi2 gate) from `side=head`/`side=arm` (ArUco pose plausibility gate) in the report. Only count head/arm rejections toward the ArUco rejection rate.

## Alternative Approaches

1. **Separate script (`aggregate_analysis.py`):** Cleaner separation but requires users to learn a new tool. Rejected because the user explicitly asked to integrate into the existing script.

2. **Post-processing existing `_analysis.txt` files:** Parse the already-generated per-log reports instead of re-parsing raw logs. Faster but loses detail (e.g., per-interval chain data is summarized away in the per-log report). Rejected because the per-log reports don't compute availability percentages yet.

3. **JSON-only output (no text report):** Useful for CI/regression but not human-readable. Rejected as sole output; included as `--aggregate-json` option alongside the text report.
