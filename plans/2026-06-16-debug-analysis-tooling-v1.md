# Debug Analysis Tooling — `analyze-log`, `analyze-bag`, `record-debug`

Status: **IMPLEMENTED** (2026-06-16)

## Objective

Add host-side analysis + instrumentation tooling so post-run triage is one command
instead of manual grepping. Three pieces:

1. `scripts/sysmon.py` — captures host + Jetson system telemetry to JSONL during a run.
2. `scripts/analyze_log.py` — parses a host log into a high-signal text report.
3. `scripts/analyze_bag.py` — parses a rosbag (+ sibling `sysmon.jsonl`) into a report.

Plus Makefile targets: `record-bag` (renamed from `record-v6`), `record-debug`,
`analyze-log`, `analyze-bag`, `analyze-bag-meta`.

## Key design decisions (final)

- **Host-side MCAP parsing.** `pip install mcap mcap-ros2-support` was installed on the
  host. This means bag analysis (including deep Tier B replay) runs **entirely on the
  host — no container needed.** This is a major UX win over the original two-tier design.
- **System telemetry co-located with the bag.** `record-debug` writes `sysmon.jsonl`
  into the *same* bag directory (`data/bags/v6_<ts>/sysmon.jsonl`), so the bag analyzer
  finds it automatically and overlays system metrics on the topic timeline.
- **Output format: JSONL.** One JSON object per line, each tagged `"source": "host"|"jetson"`.
  Appendable, streamable, parses trivially in pandas.
- **Single persistent SSH to the Jetson** for the whole session (streams `tegrastats`),
  with `sudo -S` password pattern used elsewhere in the repo.
- **No live DDS spy.** Per-topic bandwidth is derived from the recorded bag (message
  sizes + timestamps), avoiding the extra subscriber load that the network-tuning plan
  warns against.

## What each script captures

### `scripts/sysmon.py` — system telemetry capture

Runs alongside the rosbag recording. Writes JSONL to `<bag_dir>/sysmon.jsonl`.

**Host metrics (via psutil + pynvml + chronyc):**
- CPU: per-core percent + load average (1/5/15 min)
- Memory: used/available/percent
- GPU (RTX 3050): util %, VRAM used/total, temperature
- Network: per-NIC bytes_sent/bytes_recv/drops/errors (deltas per sample) — the
  **DDS-health proxy** on the 10.42 interface
- Chrony drift: system offset, last offset, RMS offset, frequency ppm (from
  `chronyc -c tracking`)

**Jetson metrics (via single persistent SSH):**
- `tegrastats` stream (sudo): CPU per-core, GPU, RAM, EMC, thermal
- `chronyc -c tracking`: drift (same fields as host)
- `/proc/net/dev`: NIC bytes/drops/errors on the Jetson side

CLI: `python3 scripts/sysmon.py --bag-dir <dir> [--no-jetson] [--interval 1.0]`

### `scripts/analyze_log.py` — host log analyzer

Pure stdlib (+ optional matplotlib for `--plot`). Default target: newest
`logs/host-log-*.txt`. Sections:

1. **Run header** — file, line count, wall-clock span, launch args, nodes started (PIDs).
2. **Severity tally** — ERROR/WARN/INFO counts per node.
3. **Known-failure digest** — pattern-matches recurring modes (TF jumps per edge, odom
   suppressions, gtsam rejections, tsdf timeouts, OpenVINS chain MISSING, cloud stalls)
   with count + first/last timestamp + root-cause hint.
4. **Diagnostics timeline** — reconstructs time-series from the 5s `[DIAG-*]` blocks:
   per-topic rates (with STARVED/FLOODED flags), clock offset, pose norms, TF chain
   flips, TF-jump bursts.
5. **First/last ERROR context.**
6. **Crash/process-death detection.**

CLI: `make analyze-log [LOG=...]` / `python3 scripts/analyze_log.py [--plot] [--all]`

### `scripts/analyze_bag.py` — rosbag + sysmon analyzer

Host-side MCAP parsing (no container). Default target: newest `data/bags/v6_*/`.

**Tier A — metadata (instant):** parses `metadata.yaml` → per-topic table with effective
rate vs nominal + health flags (ABSENT / STARVED / FLOODED).

**Tier B — deep MCAP replay:** inter-message jitter/gaps, per-topic bandwidth (MB/s),
pose-trajectory norms + jump detection, TF jump reconstruction from `/tf`.

**Sysmon overlay:** if `<bag_dir>/sysmon.jsonl` exists, loads it and prints a system
metrics summary (CPU/GPU/RAM/NIC/drift for host + Jetson) aligned to the bag timeline.
With `--plot`, overlays system metrics on the topic-rate plot.

CLI: `make analyze-bag [BAG=...]` / `python3 scripts/analyze_bag.py [--plot] [--no-deep]`

## Makefile targets

```make
record-bag:      ## Record rosbag (renamed from record-v6; alias kept)
record-debug:    ## Record rosbag + system telemetry (host + Jetson) into the bag dir
analyze-log:     ## Analyze the latest host-log-*.txt (host-side)
analyze-bag:     ## Analyze the latest bag + sibling sysmon.jsonl (host-side MCAP)
analyze-bag-meta:## Analyze the latest bag metadata only (Tier A, fast)
```

`record-v6` is kept as a backward-compatible alias for `record-bag` in both Makefiles.

## Verification (all passing)

- `make analyze-log` on `logs/host-log-20260616_162416.txt` reports all 15 nodes with
  PIDs, the TF-jump counts, tsdf timeouts, gtsam rejections, and the rate timeline.
- `make analyze-bag` on `data/bags/v6_20260616_141356/` flags `/vis/head_arm_pose`
  (count 0) as ABSENT and clouds as STARVED, with deep replay jitter/bandwidth.
- `make analyze-bag-meta` runs Tier A only (fast, no MCAP replay).
- `--plot` writes PNG timelines next to the source for both analyzers.
- `sysmon.py --no-jetson` smoke-tested: writes valid JSONL with host CPU/GPU/RAM/NIC/drift.

## Out of scope

- Live/streaming dashboards (the `pipeline_diagnostics_node` already covers live).
- Automatic root-cause fixes — these scripts diagnose, they don't patch.
